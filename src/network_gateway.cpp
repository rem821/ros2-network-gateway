/**
 * @file network_gateway.cpp
 * @brief Implementation of the NetworkGateway ROS2 node.
 *
 * Entry point for the ros2_network_gateway package. Creates the node,
 * loads parameters, discovers topics, and forwards them over UDP.
 */
#include <algorithm>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "ros2_network_gateway/network_interfaces/udp_interface.hpp"
#include "ros2_network_gateway/network_gateway.hpp"

NetworkGateway::NetworkGateway(const std::string &nodeName) : Node(nodeName) {
}

void NetworkGateway::initialize() {
    loadParameters();
    loadNetworkInterface();
    networkInterface_->open();
}

void NetworkGateway::loadParameters() {
    this->declare_parameter("network_interface", "UDP");
    this->get_parameter("network_interface", this->networkInterfaceName_);

    this->declare_parameter("topics", std::vector<std::string>{
                                "/loki_1/chassis/battery_voltage", "/loki_1/chassis/clock"
                            });
    this->get_parameter("topics", requestedTopics_);

    this->declare_parameter("topic_refresh_rate", 5000);
    this->get_parameter("topic_refresh_rate", topicRefreshRate_);

    this->declare_parameter("topic_publish_rate", 500);
    this->get_parameter("topic_publish_rate", topicPublishRate_);

    this->declare_parameter("local_address", std::string("10.0.0.230"));
    this->declare_parameter("receive_port", 8500);
    this->declare_parameter("remote_address", std::string("10.0.20.219"));
    this->declare_parameter("send_port", 8502);
    this->declare_parameter("compression_level", 0);

    this->get_parameter("local_address", localAddress_);
    this->get_parameter("receive_port", receivePort_);
    this->get_parameter("remote_address", remoteAddress_);
    this->get_parameter("send_port", sendPort_);
    int compressionParam = 0;
    this->get_parameter("compression_level", compressionParam);
    compressionLevel_ = static_cast<uint8_t>(std::clamp(compressionParam, 0, 22));

    topicCheckTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(topicRefreshRate_),
        std::bind(&NetworkGateway::subscribeTopics, this));

    networkCheckTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&NetworkGateway::checkNetworkHealth, this)
    );
}

void NetworkGateway::subscribeTopics() {
    if (!rclcpp::ok()) { return; }

    RCLCPP_INFO(get_logger(), "Subscribing to topics from the ROS2 graph ...");
    const auto allTopicsAndTypes = this->get_topic_names_and_types();

    // Track which topics are currently alive so we can prune disappeared ones below.
    std::unordered_set<std::string> currentTopics;
    for (const auto &[topicName, topicType]: allTopicsAndTypes) {
        currentTopics.insert(topicName);

        if (topicType.empty()) { continue; }

        // Whitelist filtering — empty list means "forward everything".
        if (!requestedTopics_.empty() && std::ranges::find(requestedTopics_, topicName) == requestedTopics_.end()) {
            continue;
        }
        if (subscribedTopics_.contains(topicName)) { continue; }

        RCLCPP_INFO(get_logger(), "Found topic %s of type %s", topicName.c_str(), topicType[0].c_str());
        auto manager = std::make_shared<SubscriptionManager>(this->shared_from_this(), topicName, topicType[0], compressionLevel_);
        subscriptionManagers_.emplace_back(
            topicName, manager
        );
        subscribedTopics_.insert(topicName);

        // Proto timer — sends the JSON schema every 5 s so the client knows the structure.
        networkGatewayTimers_.emplace_back(
            this->create_wall_timer(
                std::chrono::seconds(5),
                [this, manager]() {
                    sendData(manager, true);
                }
            ));
        // Data timer — sends the latest message value at the configured rate.
        networkGatewayTimers_.emplace_back(
            this->create_wall_timer(
                std::chrono::milliseconds(topicPublishRate_),
                [this, manager]() {
                    sendData(manager, false);
                }
            ));
    }

    // Prune subscriptions for topics that no longer exist on the graph.
    for (auto it = subscriptionManagers_.begin(); it != subscriptionManagers_.end();) {
        const std::string &topicName = it->first;
        if (!currentTopics.contains(topicName)) {
            RCLCPP_WARN(get_logger(), "Topic %s disappeared — unsubscribing", topicName.c_str());
            subscribedTopics_.erase(topicName);
            it = subscriptionManagers_.erase(it);
        } else
            ++it;
    }
}

void NetworkGateway::loadNetworkInterface() {
    if (networkInterfaceName_ == "UDP") {
        networkInterface_ = std::make_shared<UdpInterface>(localAddress_, remoteAddress_, receivePort_, sendPort_);
        networkInterface_->initialize(shared_from_this(),
                                      std::bind(&NetworkGateway::receiveData, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Loaded network interface: %s", networkInterfaceName_.c_str());
    }
}

void NetworkGateway::receiveData(std::span<const uint8_t> /* data */) {
    // TODO: Parse incoming messages and publish them onto ROS2 topics
}

void NetworkGateway::sendData(const std::shared_ptr<SubscriptionManager> &manager, const bool sendProto) {
    manager->checkSubscription();
    if (!manager->hasData()) {
        return;
    }
    if (!networkInterface_->is_ready()) {
        return;
    }

    std::string data;
    if (sendProto) { data = manager->getMessageProto(); } else { data = manager->getData(false); }

    if (data.empty()) { return; }

    auto now = std::chrono::system_clock::now();
    const std::string &topic = manager->getTopicName();
    const std::string &type = manager->getTopicType();
    // Proto packets are never compressed (they're small JSON schemas).
    bool compressed = !sendProto && manager->isCompressed();
    auto header = createHeader(topic, type, compressed);

    std::vector<uint8_t> message;
    message.reserve(header.size() + data.size());
    message.insert(message.end(), header.begin(), header.end());
    message.insert(message.end(), data.begin(), data.end());

    const std::string msgStr(message.begin(), message.end());
    if (sendProto) { RCLCPP_INFO(get_logger(), "Sending message proto: %s", msgStr.c_str()); } else {
        RCLCPP_INFO(get_logger(), "Sending message: %s", msgStr.c_str());
    }

    networkInterface_->write(message);
    auto end = std::chrono::system_clock::now();
    RCLCPP_DEBUG(
        this->get_logger(),
        "Send time: %f ms",
        std::chrono::duration<double, std::milli>(end - now).count());
}

void NetworkGateway::checkNetworkHealth() {
    if (!networkInterface_) {
        RCLCPP_WARN(this->get_logger(), "Network interface is null, attempting to recreate");
        loadNetworkInterface();
        if (networkInterface_) {
            networkInterface_->open();
        }
        return;
    }
    if (networkInterface_->has_failed()) {
        RCLCPP_WARN(this->get_logger(), "Network interface has failed. Resetting");
        networkInterface_->close();
        networkInterface_->open();
    }
}

std::vector<uint8_t> NetworkGateway::createHeader(const std::string &topic, const std::string &type, bool compressed) {
    // Wire format: [8B timestamp (double, LE)][1B compressed flag][topic\0][type\0]
    double current_time = rclcpp::Clock().now().seconds();
    auto current_time_bytes = std::bit_cast<std::array<uint8_t, sizeof(current_time)> >(current_time);

    std::vector<uint8_t> header;
    header.reserve(current_time_bytes.size() + 1 + topic.size() + 1 + type.size() + 1);

    header.insert(header.end(), current_time_bytes.begin(), current_time_bytes.end());
    header.push_back(compressed ? 1 : 0);

    header.insert(header.end(), topic.begin(), topic.end());
    header.push_back('\0');

    header.insert(header.end(), type.begin(), type.end());
    header.push_back('\0');
    return header;
}


void NetworkGateway::shutdown() {
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NetworkGateway>("network_gateway");
    node->initialize();
    rclcpp::spin(node);
    node->shutdown();
    node.reset();

    rclcpp::shutdown();
    return 0;
}

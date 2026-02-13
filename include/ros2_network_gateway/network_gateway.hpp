/**
 * @file network_gateway.hpp
 * @brief Main ROS2 node that bridges ROS2 topics to non-ROS2 devices over a network interface.
 *
 * NetworkGateway periodically discovers topics on the ROS2 graph, creates generic
 * subscriptions for each, and forwards the serialized-to-JSON payloads over UDP
 * (or another NetworkInterface implementation) to a remote consumer.
 *
 * Two kinds of packets are emitted per topic:
 *  - Proto packets (every 5 s): a JSON schema describing the message structure.
 *  - Data packets (at topic_publish_rate): the latest message value as JSON,
 *    optionally Zstd-compressed.
 */
#pragma once
#include <rclcpp/node.hpp>

#include "subscription_manager.hpp"
#include "network_interfaces/network_interface_base.hpp"

class NetworkGateway : public rclcpp::Node {
public:
    explicit NetworkGateway(const std::string &nodeName);

    /**
     * @brief One-time setup: load parameters, create the network interface, and open it.
     *
     * Must be called after construction (needs shared_from_this()).
     */
    void initialize();

    /** @brief Graceful teardown (reserved for future cleanup logic). */
    void shutdown();

private:
    /** @brief Declare and read all ROS2 parameters, then start the periodic timers. */
    void loadParameters();

    /**
     * @brief Timer callback: re-scan the ROS2 graph and manage subscriptions.
     *
     * New topics matching the whitelist get a SubscriptionManager plus two send
     * timers (proto and data). Topics that have disappeared are cleaned up.
     */
    void subscribeTopics();

    /** @brief Instantiate the configured NetworkInterface (currently only UDP). */
    void loadNetworkInterface();

    /**
     * @brief Callback invoked by the network interface when a UDP packet arrives.
     *
     * @note Not yet implemented — reserved for future network-to-ROS2 publishing.
     */
    void receiveData(std::span<const uint8_t> data);

    /**
     * @brief Assemble a header + payload packet and send it through the network interface.
     *
     * @param manager  The SubscriptionManager holding the latest data for a topic.
     * @param sendProto If true, send the message schema; otherwise send the latest data.
     */
    void sendData(const std::shared_ptr<SubscriptionManager> &manager, const bool sendProto = true);

    /**
     * @brief Timer callback: check and recover the network interface if it has failed.
     *
     * If the interface pointer is null it is recreated via loadNetworkInterface().
     * If it reports has_failed(), it is closed and reopened.
     */
    void checkNetworkHealth();

    /**
     * @brief Build the binary header prepended to every UDP packet.
     *
     * Layout: [8-byte double timestamp][1-byte compression flag][topic\\0][type\\0]
     *
     * @param topic      ROS2 topic name (e.g. "/imu/data").
     * @param type       ROS2 message type (e.g. "sensor_msgs/msg/Imu").
     * @param compressed Whether the payload that follows is Zstd-compressed.
     * @return The header as a byte vector.
     */
    std::vector<uint8_t> createHeader(const std::string &topic, const std::string &type, bool compressed);

    // -- Parameters --
    std::string networkInterfaceName_;
    std::vector<std::string> requestedTopics_;   ///< Topic whitelist; empty = forward all.
    uint16_t topicRefreshRate_ = 5000;           ///< Graph re-scan interval (ms).
    uint16_t topicPublishRate_ = 500;            ///< Per-topic data send interval (ms).

    // -- Subscription bookkeeping --
    std::vector<std::pair<std::string, std::shared_ptr<SubscriptionManager> > > subscriptionManagers_;
    std::unordered_set<std::string> subscribedTopics_;

    rclcpp::TimerBase::SharedPtr topicCheckTimer_;

    // -- Network --
    std::shared_ptr<NetworkInterface> networkInterface_;
    rclcpp::TimerBase::SharedPtr networkCheckTimer_;

    /// Per-topic timers (proto + data) — kept alive as long as the topic exists.
    std::vector<rclcpp::TimerBase::SharedPtr> networkGatewayTimers_;

    std::string localAddress_, remoteAddress_;
    uint16_t receivePort_, sendPort_;
    uint8_t compressionLevel_ = 0;              ///< Zstd level (0 = disabled, 1-22).
};

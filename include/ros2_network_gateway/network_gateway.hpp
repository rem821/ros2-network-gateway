//
// Created by standa on 10/2/25.
//
#pragma once
#include <rclcpp/node.hpp>

#include "subscription_manager.hpp"
#include "network_interfaces/network_interface_base.hpp"

class NetworkGateway : public rclcpp::Node {
public:
    explicit NetworkGateway(const std::string &nodeName);

    void initialize();


    void shutdown();

private:
    void loadParameters();

    void subscribeTopics();

    void loadNetworkInterface();

    void receiveData(std::span<const uint8_t> data);

    void sendData(const std::shared_ptr<SubscriptionManager> &manager, const bool sendProto = true);

    void checkNetworkHealth();

    std::vector<uint8_t> createHeader(const std::string &topic, const std::string &type);

    std::string networkInterfaceName_;
    std::vector<std::string> requestedTopics_;
    uint16_t topicRefreshRate_ = 5000;
    uint16_t topicPublishRate_ = 500;

    std::vector<std::pair<std::string, std::shared_ptr<SubscriptionManager> > > subscriptionManagers_;
    std::unordered_set<std::string> subscribedTopics_;

    rclcpp::TimerBase::SharedPtr topicCheckTimer_;

    std::shared_ptr<NetworkInterface> networkInterface_;
    rclcpp::TimerBase::SharedPtr networkCheckTimer_;

    std::vector<rclcpp::TimerBase::SharedPtr> networkGatewayTimers_;

    std::string localAddress_, remoteAddress_;
    uint16_t receivePort_, sendPort_;
};

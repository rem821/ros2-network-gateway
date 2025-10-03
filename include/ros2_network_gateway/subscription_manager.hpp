//
// Created by standa on 10/2/25.
//
#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include <rclcpp/node.hpp>

class SubscriptionManager
{
public:
    SubscriptionManager(rclcpp::Node::SharedPtr node, std::string topicName,
                        std::string topicType, uint8_t compressionLevel);

    void checkSubscription();
    [[nodiscard]] bool hasData() const { return hasData_; }

    // std::vector<uint8_t> getData(bool wipe)
    // {
    //     std::vector<uint8_t> data = data_;
    //     if (wipe) { data_.clear(); hasData_ = false; }
    //     return data;
    // }

    std::string getData(bool wipe)
    {
        std::string data = data_;
        if (wipe) { data_.clear(); hasData_ = false; }
        return data;
    }

    [[nodiscard]] std::string getTopicName() const { return topicName_; }
    [[nodiscard]] std::string getTopicType() const { return topicType_; }

private:
    void subscribe();
    void handleMessage(const rclcpp::Node::SharedPtr& node, const std::string& topic, const std::string& type,
                       const std::shared_ptr<const rclcpp::SerializedMessage>& serializedMsg);

    nlohmann::json memberToJson(const void* field, const rosidl_typesupport_introspection_cpp::MessageMember& member);
    nlohmann::json messageToJson(const void* msg, const rosidl_typesupport_introspection_cpp::MessageMembers* members);
    static nlohmann::json rosTypeToJson(const rosidl_typesupport_introspection_cpp::MessageMember* member, const void* elem);

    [[nodiscard]] std::vector<uint8_t> compress(std::vector<uint8_t> const& data) const;
    static std::vector<uint8_t> decompress(std::span<const uint8_t> compressedData);

    rclcpp::Node::SharedPtr node_;
    std::string topicName_;
    std::string topicType_;
    uint8_t compressionLevel_;

    rclcpp::GenericSubscription::SharedPtr subscriber_;

    bool hasData_ = false;
    //std::vector<uint8_t> data_;
    std::string data_;
};

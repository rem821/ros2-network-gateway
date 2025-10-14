== ROS2 Network Gateway

=== This package is a part of my thesis, so it is a part of an active research. Due to this, it lacks documentation and it isn't versioned *yet*.

ROS2 Network Gateway is a lightweight package that allows non-ROS2 devices of any kind consume ROS2 topics.

Since compiling the whole DDS implementation is not an option for most use-cases, this package provides a translation layer to a simple UDP communication.

The package is able to subscribe to all topics available on the ROS2 network dynamically by making use of the 'get_topic_names_and_types' and 'create_generic_subscription' functions available in the Node API.
This means you won't need to manually add topics to the package configuration file every time your system changes. You also won't need to include every interface possible as this is done by using the 'rosidl_typesupport_introspection_cpp' during runtime.

If you want to limit the scope of topics going through the gateway, you can either limit it to a specific namespace, whitelist or blacklist specific topics or use regex.

The current implementation state only allows for a uni-directional communication, but the support for receiving UDP messages and converting them to ROS2 shall come later.
The client code is currently only part of the BUT_Telepresence (https://github.com/rem821/BUT_Telepresence) repo, but will be abstracted away and ported here in the future.

cmake_minimum_required(VERSION 3.10.2)
project(separable_jaws_config)
find_package(ament_cmake REQUIRED)

ament_package()

# install launch scripts
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

# install moveit2 config files
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

install(FILES .setup_assistant
        DESTINATION share/${PROJECT_NAME}
)

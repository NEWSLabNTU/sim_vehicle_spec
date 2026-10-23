cmake_minimum_required(VERSION 3.14)
project(sim_vehicle_spec)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The specs, the outputs generated here and the SSv2 catalog, under share/sim_vehicle_spec;
# the library importable for other generators.
install(DIRECTORY vehicles generated catalogs DESTINATION share/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})

ament_package()

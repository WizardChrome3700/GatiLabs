cmake_minimum_required(VERSION 3.15)
project(QuadcopterSim VERSION 1.0 LANGUAGES CXX)

# Define root include paths so out-of-source builds (e.g., inside a build/ dir) don't break
set(PROJECT_INCLUDES ${CMAKE_SOURCE_DIR}/include)
set(JSON4CPP_INCLUDES ${CMAKE_SOURCE_DIR}/external/nlohmann)
set(EIGEN_INCLUDES ${CMAKE_SOURCE_DIR}/external/eigen-5.0.0)

# --- 1. Base Physics & Aerodynamics ---
add_library(physics_engine src/physics/RigidBody.cpp)
target_include_directories(physics_engine PUBLIC ${PROJECT_INCLUDES} ${PROJECT_INCLUDES}/physics ${EIGEN_INCLUDES})

add_library(BETRotor src/BETRotor/BETRotor.cpp)
target_include_directories(BETRotor PUBLIC ${PROJECT_INCLUDES} ${PROJECT_INCLUDES}/BETRotor ${EIGEN_INCLUDES})

# --- 2. The Powertrain Components ---
add_library(Battery src/Battery/LipoBattery.cpp)
target_include_directories(Battery PUBLIC ${PROJECT_INCLUDES} ${PROJECT_INCLUDES}/Battery ${EIGEN_INCLUDES})

add_library(ESC src/ESC/ESC.cpp)
target_include_directories(ESC PUBLIC ${PROJECT_INCLUDES} ${PROJECT_INCLUDES}/ESC ${EIGEN_INCLUDES})

add_library(BLDCMotor src/Motor/BLDC_Motor.cpp)
target_include_directories(BLDCMotor PUBLIC ${PROJECT_INCLUDES} ${PROJECT_INCLUDES}/Motor ${EIGEN_INCLUDES})
target_link_libraries(BLDCMotor PUBLIC BETRotor)

# --- 3. The Vehicles (Chassis & Assembly) ---
add_library(Vehicles src/Vehicles/Vehicle.cpp src/Vehicles/Quadcopter.cpp)
target_include_directories(Vehicles PUBLIC ${PROJECT_INCLUDES} ${PROJECT_INCLUDES}/Vehicles ${EIGEN_INCLUDES})
target_link_libraries(Vehicles PUBLIC physics_engine Battery ESC BLDCMotor)

add_library(VehicleFactory src/Vehicles/VehicleFactory.cpp)
target_include_directories(VehicleFactory PUBLIC ${PROJECT_INCLUDES} ${PROJECT_INCLUDES}/Vehicles ${EIGEN_INCLUDES} ${JSON4CPP_INCLUDES})
target_link_libraries(VehicleFactory PUBLIC Vehicles)

# --- 4. The Data Logger ---
add_library(DataLogger src/DataLogger/DataLogger.cpp)
target_include_directories(DataLogger PUBLIC ${PROJECT_INCLUDES} ${PROJECT_INCLUDES}/DataLogger ${EIGEN_INCLUDES})
target_link_libraries(DataLogger PUBLIC Vehicles)

# --- 5. The Executable ---
add_executable(quad_sim src/main.cpp)
target_link_options(quad_sim PRIVATE -static)
target_link_libraries(quad_sim Vehicles DataLogger VehicleFactory)
target_include_directories(quad_sim PUBLIC ${PROJECT_INCLUDES} ${EIGEN_INCLUDES} ${JSON4CPP_INCLUDES})
# Build test_nao_phase_provider
ament_add_gtest(test_nao_phase_provider
    test_nao_phase_provider.cpp)

target_link_libraries(test_nao_phase_provider
    ${PROJECT_NAME}_node
)
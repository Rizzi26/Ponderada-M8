cmake_minimum_required(VERSION 3.8)
project(cg_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontre dependências
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)  # Adiciona std_msgs para dependências do serviço, se necessário

# Adiciona a definição do arquivo de serviço
find_package(rosidl_default_generators REQUIRED)

# Define os arquivos de serviço que queremos compilar
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveCmd.srv"
  DEPENDENCIES std_msgs  # Ou outras dependências, se o serviço utilizar mensagens específicas
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # As linhas abaixo ignoram verificações de copyright e cpplint (se aplicável)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(skidpad)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Encontrar dependências ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lart_msgs REQUIRED)

# --- Criar o Executável ---
# NOTA: Removida a vírgula que estava entre os ficheiros fonte
add_executable(skidpad_exec 
  src/my_node.cpp
  src/utils.cpp
)

# --- Incluir Cabeçalhos (Headers) ---
# Necessário para que os ficheiros .cpp encontrem a pasta include/
target_include_directories(skidpad_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Adicionar as dependências ao executável ---
ament_target_dependencies(skidpad_exec
  rclcpp
  visualization_msgs
  nav_msgs
  tf2
  std_msgs
  lart_msgs
)

# --- Instalação (Install) ---

# 1. Instalar o executável na pasta lib do pacote
install(TARGETS skidpad_exec
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Instalar a pasta include
install(DIRECTORY include/
  DESTINATION include
)

# 3. Instalar o ficheiro de dados (CSV)
# Isto permite-te ler o skidpad_path.csv usando a diretoria de partilha do ament (ament_index_cpp::get_package_share_directory("skidpad"))
install(FILES src/skidpad_path.csv
  DESTINATION share/${PROJECT_NAME}/data
)

# --- Testes (Ignorados no build principal) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
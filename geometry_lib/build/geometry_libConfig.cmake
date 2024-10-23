
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was geometry_libConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include(CMakeFindDependencyMacro)

# Trova le dipendenze richieste
find_dependency(rclcpp)
find_dependency(std_msgs)
find_dependency(geometry_msgs)
find_dependency(Eigen3)
find_dependency(kdl_parser)

# Specifica le directory di inclusione
set(geometry_lib_INCLUDE_DIRS "/home/arise/libraries/geometry_lib/include")

# Specifica il target della libreria
set(geometry_lib_LIBRARIES "${CMAKE_INSTALL_PREFIX}/lib/libgeometry_lib.so")

# Fornisci le variabili al pacchetto
mark_as_advanced(geometry_lib_INCLUDE_DIRS geometry_lib_LIBRARIES)

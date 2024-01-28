# Auto add library
# Usage: auto_add_library(lib_target 
#                         [SOURCES] src1 [src2 ...] 
#                         [INSTALL_DIR dst_dir] 
#                         [INCLUDES dir1 ...] 
#                         [LIBRARIES lib1 ...] 
#                         [DEFINITIONS def1 ...])
#                         [NAMESPACE namespace])
#
# "lib_target" is the library target name, The target name should be 
#     unique in a cmake project. 
# "SOURCES" keyword is used to specify source files of the target. SOURCES 
#     keyword can be omitted.
# "INSTALL_DIR" keyword is used to specify the installation directory of the 
#     target. The install directory is forced relative to $AUTO_ROOT. 
#     If INSTALL_DIR is not specified, this library will not be installed.
# "INCLUDES" keyword is used to specify include direcotries.
# "LIBRARIES" keyword is used to specify link libraries
# "DEFINITIONS" keyword is used to specify target definitions.
# "NAMESPACE" namespace for exported target
# "PATHS" keyword is used to specify paths for installed target. Is not specified,
#          ${AUTO_ROOT}/lib and ${AUTO_ROOT}/third_party/lib will be used.
#
function(auto_add_library target)
    cmake_parse_arguments(AUTO_ADD_LIBRARY "" "INSTALL_DIR;INSTALL_EXPORT_DIR;NAMESPACE;EXPORT_COMPONENT""SOURCES;INCLUDES;LIBRARIES;DEFINITIONS;PATHS" ${ARGN})
       
    # Check if target name is specified
    if(NOT target)
        message(FATAL_ERROR "Target name not specified!")
    endif(NOT target)

    if(IS_ABSOLUTE ${AUTO_ADD_LIBRARY_INSTALL_DIR})
        MESSAGE(FATAL_ERROR "install  dirs only accept relative path!")
    endif()

    if(NOT AUTO_ADD_LIBRARY_PATHS)
        set(AUTO_ADD_LIBRARY_PATHS ${AUTO_ROOT}/lib ${AUTO_ROOT}/third_party/lib)
    endif()

    # Check if source files are specifed
    if(NOT AUTO_ADD_LIBRARY_SOURCES)
        if(NOT AUTO_ADD_LIBRARY_UNPARSED_ARGUMENTS)
            set(LIBRARY_TYPE "INTERFACE")
        endif(NOT AUTO_ADD_LIBRARY_UNPARSED_ARGUMENTS)
        set(_sources ${AUTO_ADD_LIBRARY_UNPARSED_ARGUMENTS})
    else(NOT AUTO_ADD_LIBRARY_SOURCES)
        set(_sources ${AUTO_ADD_LIBRARY_SOURCES})
    endif(NOT AUTO_ADD_LIBRARY_SOURCES)

    if(LIBRARY_TYPE STREQUAL "INTERFACE")
        set(DEPENDENCY_TYPE "INTERFACE")
    else()
        set(DEPENDENCY_TYPE "PUBLIC")
    endif()

    add_library(${target} ${_sources} ${LIBRARY_TYPE})
    target_include_directories(${target} ${DEPENDENCY_TYPE} ${AUTO_ADD_LIBRARY_INCLUDES})
    target_link_libraries(${target} ${DEPENDENCY_TYPE} ${AUTO_ADD_LIBRARY_LIBRARIES})
    target_compile_definitions(${target} ${DEPENDENCY_TYPE} ${AUTO_ADD_LIBRARY_DEFINITIONS})

    if(AUTO_ADD_LIBRARY_INSTALL_DIR)
        #calculate rpath
        if(NOT LIBRARY_TYPE STREQUAL "INTERFACE")
            foreach(input_rpath ${AUTO_ADD_LIBRARY_PATHS})
                if(IS_ABSOLUTE ${input_rpath})
                    file(RELATIVE_PATH actual_rpath ${CMAKE_INSTALL_PREFIX}/${AUTO_ADD_LIBRARY_INSTALL_DIR} ${input_rpath})
                    set_property(TARGET ${target} APPEND PROPERTY INSTALL_RPATH "\$ORIGIN/${actual_rpath}")
                else()
                    set_property(TARGET ${target} APPEND PROPERTY INSTALL_RPATH "\$ORIGIN/${input_rpath}")
                endif()
            endforeach()
            set_target_properties(${target} PROPERTIES 
                LIBRARY_OUTPUT_DIRECTORY ${AUTO_ADD_LIBRARY_INSTALL_DIR})
        endif()

        if (AUTO_ADD_LIBRARY_EXPORT_COMPONENT)
            set(EXP_COMP ${AUTO_ADD_LIBRARY_EXPORT_COMPONENT}-)
        endif()

        if (NOT AUTO_ADD_LIBRARY_INSTALL_EXPORT_DIR)
            set(AUTO_ADD_LIBRARY_INSTALL_EXPORT_DIR ${AUTO_ADD_LIBRARY_INSTALL_DIR}/cmake/${PROJECT_NAME})
        endif()
    
        install(TARGETS ${target} 
            EXPORT ${CMAKE_PROJECT_NAME}-${EXP_COMP}targets 
            DESTINATION ${AUTO_ADD_LIBRARY_INSTALL_DIR}
            COMPONENT runtime)
        install(EXPORT ${CMAKE_PROJECT_NAME}-${EXP_COMP}targets 
            NAMESPACE "${AUTO_ADD_LIBRARY_NAMESPACE}"
            DESTINATION ${AUTO_ADD_LIBRARY_INSTALL_EXPORT_DIR}
            COMPONENT development)
    endif(AUTO_ADD_LIBRARY_INSTALL_DIR)

endfunction(auto_add_library)


# Auto add executable
# Usage: AUTO_ADD_EXECUTABLE_APP(executable_target 
#                         [SOURCES] src1 [src2 ...] 
#                         [INSTALL_DIR dst_dir] 
#                         [INCLUDES dir1 ...] 
#                         [LIBRARIES lib1 ...] 
#                         [DEFINITIONS def1 ...]
#
# "executable_target" is the executable_target target name, The target name should be 
#     unique in a cmake project. 
# "SOURCES" keyword is used to specify source files of the target. SOURCES 
#     keyword can be omitted.
# "INSTALL_DIR" keyword is used to specify the installation directory of the 
#     target. The install directory is forced relative to $AUTO_ROOT. 
#     If INSTALL_DIR is not specified, this target will not be installed.
# "INCLUDES" keyword is used to specify include direcotries.
# "LIBRARIES" keyword is used to specify link libraries
# "DEFINITIONS" keyword is used to specify target definitions.
#
function(auto_add_executable target)
    cmake_parse_arguments(AUTO_ADD_EXECUTABLE "" "INSTALL_DIR;NAMESPACE" 
        "SOURCES;INCLUDES;LIBRARIES;DEFINITIONS,RPATHS" ${ARGN})

    # Check if target name is specified
    if(NOT target)
        MESSAGE(FATAL_ERROR "Target name not specified!")
    endif(NOT target)

    if(IS_ABSOLUTE ${AUTO_ADD_EXECUTABLE_INSTALL_DIR})
        message(FATAL_ERROR "INSTALL_DIR only accept relative path!")
    endif()

    if(NOT AUTO_ADD_EXECUTABLE_RPATHS)
        set(AUTO_ADD_EXECUTABLE_RPATHS ${AUTO_ROOT}/lib ${AUTO_ROOT}/third_party/lib)
    endif()

    # Check if source files are specifed
    if(NOT AUTO_ADD_EXECUTABLE_SOURCES)
        if(NOT AUTO_ADD_EXECUTABLE_UNPARSED_ARGUMENTS)
            message(FATAL_ERROR 
                "No source file specified for target ${target}")
        endif(NOT AUTO_ADD_EXECUTABLE_UNPARSED_ARGUMENTS)
        set(_sources ${AUTO_ADD_EXECUTABLE_UNPARSED_ARGUMENTS})
    else(NOT AUTO_ADD_EXECUTABLE_SOURCES)
        set(_sources ${AUTO_ADD_EXECUTABLE_SOURCES})
    endif(NOT AUTO_ADD_EXECUTABLE_SOURCES)

    add_executable(${target} ${_sources})
    target_include_directories(${target} PRIVATE ${AUTO_ADD_EXECUTABLE_INCLUDES})
    target_link_libraries(${target} PRIVATE ${AUTO_ADD_EXECUTABLE_LIBRARIES})
    target_compile_definitions(${target} PRIVATE ${AUTO_ADD_EXECUTABLE_DEFINITIONS})

    if(AUTO_ADD_EXECUTABLE_INSTALL_DIR)
        #calculate rpath
        foreach(input_rpath ${AUTO_ADD_EXECUTABLE_RPATHS})
            if(IS_ABSOLUTE ${input_rpath})
                file(RELATIVE_PATH actual_rpath ${CMAKE_INSTALL_PREFIX}/${AUTO_ADD_EXECUTABLE_INSTALL_DIR} ${input_rpath})
                set_property(TARGET ${target} APPEND PROPERTY INSTALL_RPATH "\$ORIGIN/${actual_rpath}")
            else()
                set_property(TARGET ${target} APPEND PROPERTY INSTALL_RPATH "\$ORIGIN/${input_rpath}")
            endif()
        endforeach()
        set_target_properties(${target} PROPERTIES 
            RUNTIME_OUTPUT_DIRECTORY "${AUTO_ADD_EXECUTABLE_INSTALL_DIR}")
    
        install(TARGETS ${target} DESTINATION ${AUTO_ADD_EXECUTABLE_INSTALL_DIR} COMPONENT runtime)
    endif(AUTO_ADD_EXECUTABLE_INSTALL_DIR)

endfunction(auto_add_executable)


# Auto install headers
# Usage: auto_install_headers(directory [DESTINATION dest]
#                                                               [TYPE  type] )
# "directory" is the directory of  
# "DESTINATION" keyword is used to specify application installation directory.
#     The installation directory is relative to $AUTO_ROOT.
#     If DESTINATION is not specified, application will install to default
#
function(auto_install_headers) 
cmake_parse_arguments(AUTO_INSTALL_HEADERS "" "" "TYPE" ${ARGN}) 
# install header
if(NOT AUTO_INSTALL_HEADERS_TYPE)
 message(
    FATAL_ERROR
         "PROJECT TYPE NOT NOT GIVEN,CHECK UPPER PROJECT DIR"
 )
endif()

 # the "/" at the end of the path matters!
if(AUTO_INSTALL_HEADERS_TYPE STREQUAL "cyber")
     set(DEST_DIR ${AUTO_ROOT}/include)
elseif(AUTO_INSTALL_HEADERS_TYPE STREQUAL "modules")
     set(DEST_DIR ${AUTO_ROOT}/include/modules
     )
endif()

 install(
     DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
     DESTINATION ${DEST_DIR}
     FILES_MATCHING
     PATTERN "*.h"
     PATTERN "*.hpp"
     PATTERN "*.hh"
     PATTERN "*.cuh")

endfunction(auto_install_headers)


# Add  test programs
#
# Usage: auto_add_test(main 
#                         [SOURCES additional_sources ...]
#                         [INCLUDES dirs ...]
#                         [LIBRARIES libs ...]
#                         [DEFINITIONS defs ...]
#                         )
#
# main: main function source file(with extension)
# SOURCES: keyword used to specify addtional source files
# INCLUDES: keyword used to specify include directories
# LIBRARIES: keyword used to specify link libraries
# DEFINITIONS: keyword used to specify compiler definitions
FUNCTION(auto_add_test main)
    cmake_parse_arguments(COMPILE "" "" "SOURCES;LIBRARIES;INCLUDES;DEFINITIONS" ${ARGN})

    if(NOT main)
        MESSAGE(FATAL_ERROR "Main source file not given")
    endif()

    get_filename_component(target ${main} NAME_WE)
    get_filename_component(path ${main} DIRECTORY)

    if(NOT IS_ABSOLUTE path)
        set(path ${CMAKE_CURRENT_SOURCE_DIR}/${path})
    endif()
    file(RELATIVE_PATH relpath ${CMAKE_SOURCE_DIR} ${path})
    string(REPLACE "/" "_" target_prefix ${relpath})

    set(target_name ${PROJECT_NAME}_${target})

    add_executable(${target_name} ${main} ${COMPILE_SOURCES})
    target_link_libraries(${target_name} PRIVATE ${COMPILE_LIBRARIES})
    target_include_directories(${target_name} PRIVATE ${COMPILE_INCLUDES})
    target_compile_definitions(${target_name} PRIVATE ${COMPILE_DEFINITIONS})
  #[[ set_property(TARGET ${target_name} PROPERTY RUNTIME_OUTPUT_DIRECTORY ${relpath})
    set_property(TARGET ${target_name} PROPERTY RUNTIME_OUTPUT_NAME ${target})
   file(RELATIVE_PATH test_name ${CMAKE_BINARY_DIR} ${PROJECT_BINARY_DIR}/${relpath}/${target})
   message("path=${test_name}")
    add_test(NAME ${test_name} COMMAND ${relpath}/${target} WORKING_DIRECTORY ${PROJECT_BINARY_DIR})]]
ENDFUNCTION()
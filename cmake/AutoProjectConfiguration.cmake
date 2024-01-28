# The following environment variable affect the output of this script.
#    AUTO_ROOT             -- Root directory of auto project files.
#    AUTO_3RDPARTY_ROOT    -- Root directory of auto third_party project files.
#    
# The script defines the following variables:
#    AUTO_ROOT             -- Root directory of auto project files. (Default /opt/auto)
#    AUTO_3RDPARTY_ROOT    -- Rood directory of auto third_party project files. (Default ${AUTO_ROOT}/third_party)
#    CMAKE_INSTALL_PREFIX  -- Install prefix for auto project. (Default ${AUTO_ROOT}).
#    CMAKE_PREFIX_PATH     -- Additional prefix paths used for cmake to find packages. (Default "${AUTO_ROOT};${AUTO_3RDPARTY_ROOT}")
#
#!!!!!!!!!!!!!!!!!!!!!!!don't revise this file!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

unset(_DEFAULT_AUTO_ROOT)
set(_DEFAULT_AUTO_ROOT "${CMAKE_SYSROOT}/opt/auto")

macro(_is_directory output dir)
    if(${dir} MATCHES "^(.*)/([^/]+)$")
        set(${output} TRUE)
        MESSAGE("${dir} is a directory")
    else()
        set(${output} FALSE)
        MESSAGE("${dir} is not a directory")
    endif()
endmacro()

if(NOT AUTO_ROOT)
    if(DEFINED ENV{AUTO_ROOT})
        _is_directory(_IS_DIRECTORY $ENV{AUTO_ROOT})
        if(_IS_DIRECTORY)
            set(AUTO_ROOT $ENV{AUTO_ROOT})
            MESSAGE(STATUS "Set auto root directory from environment variable AUTO_ROOT:$ENV{AUTO_ROOT}")
        else()
            MESSAGE(WARNING "Environment variable AUTO_ROOT is set to $ENV{AUTO_ROOT},"
                " but it is not a valid directory! Will set AUTO_ROOT to default value \"${_DEFAULT_AUTO_ROOT}\"")
            set(AUTO_ROOT ${_DEFAULT_AUTO_ROOT})
        endif()
    else()
        set(AUTO_ROOT ${_DEFAULT_AUTO_ROOT})
    endif()
else()
    _is_directory(_IS_DIRECTORY ${AUTO_ROOT})
    if(NOT _IS_DIRECTORY)
        MESSAGE(WARNING "AUTO_ROOT is set to \"${AUTO_ROOT}\" but it is not a valid directory!"
            " Will set AUTO_ROOT to default value \"${_DEFAULT_AUTO_ROOT}\"")
        set(AUTO_ROOT ${_DEFAULT_AUTO_ROOT})
    endif()
endif()

if(NOT AUTO_3RDPARTY_ROOT)
    if(DEFINED ENV{AUTO_3RDPARTY_ROOT})
        _is_directory(_IS_DIRECTORY $ENV{AUTO_3RDPARTY_ROOT})
        if(_IS_DIRECTORY)
            set(AUTO_3RDPARTY_ROOT $ENV{AUTO_3RDPARTY_ROOT})
            MESSAGE(STATUS "Set auto 3rdparty root directory from environment variable AUTO_3RDPARTY_ROOT:$ENV{AUTO_3RDPARTY_ROOT}")
        else()
            MESSAGE(WARNING "Environment variable AUTO_3RDPARTY_ROOT is set to $ENV{AUTO_3RDPARTY_ROOT},"
                " but it is not a valid directory! Will set AUTO_3RDPARTY_ROOT to default value \"${_DEFAULT_AUTO_ROOT}/third_party\"")
            set(AUTO_3RDPARTY_ROOT ${AUTO_ROOT}/third_party)
        endif()
    else()
        set(AUTO_3RDPARTY_ROOT ${AUTO_ROOT}/third_party)
    endif()
else()
    _is_directory(_IS_DIERCTORY ${AUTO_3RDPARTY_ROOT})
    if(NOT _IS_DIRECTORY)
        MESSAGE(WARNING "AUTO_3RDPARTY_ROOT is set to \"${AUTO_3RDPARTY_ROOT}\" but it is not a valid directory!"
            " Will set AUTO_3RDPARTY_ROOT to default value \"${_DEFAULT_AUTO_ROOT}/third_party\"")
        set(AUTO_3RDPARTY_ROOT ${AUTO_ROOT}/third_party)
    endif()
endif()

set(CMAKE_INSTALL_PREFIX ${AUTO_ROOT})
set(CMAKE_PREFIX_PATH ${AUTO_ROOT};${AUTO_3RDPARTY_ROOT};$ENV{CMAKE_PREFIX_PATH})
set(3RD_PARTY_INCLUDE_DIR ${AUTO_3RDPARTY_ROOT}/include)
STRING(REPLACE ":" ";" CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}")

MESSAGE(STATUS "AUTO_ROOT=${AUTO_ROOT}")
MESSAGE(STATUS "AUTO_3RDPARTY_ROOT=${AUTO_3RDPARTY_ROOT}")

unset(_IS_DIRECTORY)
unset(_DEFAULT_AUTO_ROOT)

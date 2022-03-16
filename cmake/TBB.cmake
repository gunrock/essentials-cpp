include(FetchContent)
set(FETCHCONTENT_QUIET ON)

message(STATUS "Cloning External Project: TBB")
get_filename_component(FC_BASE "../externals"
                REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
set(FETCHCONTENT_BASE_DIR ${FC_BASE})

# Manually set the URLs from https://github.com/oneapi-src/oneTBB/releases/
if (MSVC)
  set(DOWNLOAD_URL https://github.com/oneapi-src/oneTBB/releases/download/v2021.5.0/oneapi-tbb-2021.5.0-win.zip)
  set(DOWNLOAD_HASH 096c004c7079af89fe990bb259d58983b0ee272afa3a7ef0733875bfe09fcd8e)
else()
  set(DOWNLOAD_URL https://github.com/oneapi-src/oneTBB/releases/download/v2021.5.0/oneapi-tbb-2021.5.0-lin.tgz)
  set(DOWNLOAD_HASH 74861b1586d6936b620cdab6775175de46ad8b0b36fa6438135ecfb8fb5bdf98)
endif()

FetchContent_Declare(
  TBB
    URL       ${DOWNLOAD_URL}
    URL_HASH  SHA256=${DOWNLOAD_HASH}
)

FetchContent_MakeAvailable(TBB)
set(TBB_SOURCE_DIR ${FETCHCONTENT_BASE_DIR}/tbb-src)
set(TBB_INCLUDE_DIR ${FETCHCONTENT_BASE_DIR}/tbb-src/include)
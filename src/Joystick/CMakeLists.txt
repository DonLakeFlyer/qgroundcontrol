# ============================================================================
# Joystick Module
# Game controller and joystick input support
# ============================================================================

target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE
        Joystick.cc
        Joystick.h
        JoystickManager.cc
        JoystickManager.h
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# HACK: AX12 debugging - SDL pulled out completely, so the SDL joystick backend is disabled
# target_sources(${CMAKE_PROJECT_NAME}
#     PRIVATE
#         JoystickSDL.cc
#         JoystickSDL.h
# )

file(GLOB CLI_DEMO_SOURCE_FILES
    LIST_DIRECTORIES false
    cli_demo.cpp
    )

add_executable(cli_demo
    ${CLI_DEMO_SOURCE_FILES}
    )

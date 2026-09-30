function(add_deploy_target TARGET PATH_UF2)
    add_custom_target("${TARGET}---deploy"
            DEPENDS ${TARGET}
            DEPENDS ${PATH_UF2}
            COMMAND ${CMAKE_SOURCE_DIR}/scripts/deploy.sh ${PATH_UF2}
            )
endfunction()


# Populate a git submodule the first time the build needs it.
#
# ROOT is the checkout containing the submodule, SUBMODULE its path relative to
# ROOT and MARKER a file inside it that the build is about to use. Nothing
# happens while MARKER exists, so an initialised checkout never triggers a
# fetch; callers pass a build-elsewhere override by not calling this at all.
# A failed fetch is only warned about here: the caller's own existence check
# turns it into the configure error, which then names the manual fix.
function(init_submodule_if_needed ROOT SUBMODULE MARKER)
    if(EXISTS "${ROOT}/${SUBMODULE}/${MARKER}")
        return()
    endif()
    if(NOT EXISTS "${ROOT}/.git")
        message(STATUS "${SUBMODULE} is empty and ${ROOT} is not a git checkout, so it cannot be fetched")
        return()
    endif()
    find_program(GIT_EXECUTABLE git)
    if(NOT GIT_EXECUTABLE)
        message(STATUS "${SUBMODULE} is empty and git was not found, so it cannot be fetched")
        return()
    endif()
    message(STATUS "${SUBMODULE} is empty: running git submodule update --init -- ${SUBMODULE}")
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" submodule update --init -- "${SUBMODULE}"
        WORKING_DIRECTORY "${ROOT}"
        RESULT_VARIABLE rc
    )
    if(NOT rc EQUAL 0)
        message(WARNING "git submodule update --init -- ${SUBMODULE} failed with status ${rc}")
    endif()
endfunction()

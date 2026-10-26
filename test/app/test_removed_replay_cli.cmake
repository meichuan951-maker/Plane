# Exercise argument rejection before configuration loading or device startup.
if(NOT DEFINED DEMO_AIM_BINARY OR NOT EXISTS "${DEMO_AIM_BINARY}")
  message(FATAL_ERROR "Set DEMO_AIM_BINARY to the built demo_aim_cpp executable")
endif()

execute_process(COMMAND "${DEMO_AIM_BINARY}" --help
  RESULT_VARIABLE help_result OUTPUT_VARIABLE help_output ERROR_VARIABLE help_error
  TIMEOUT 5)
if(NOT help_result STREQUAL "0")
  message(FATAL_ERROR "--help failed (${help_result}): ${help_error}")
endif()
string(TOLOWER "${help_output}${help_error}" help_text)
if(help_text MATCHES "replay|--manifest")
  message(FATAL_ERROR "--help still advertises PNG replay: ${help_text}")
endif()

function(expect_rejected expected_error)
  execute_process(COMMAND "${DEMO_AIM_BINARY}" ${ARGN}
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error
    TIMEOUT 5)
  if(NOT result STREQUAL "1")
    message(FATAL_ERROR "Expected argument rejection for ${ARGN}, got ${result}: ${output}${error}")
  endif()
  string(FIND "${output}${error}" "${expected_error}" error_position)
  if(error_position EQUAL -1)
    message(FATAL_ERROR "Wrong rejection for ${ARGN}; expected '${expected_error}': ${output}${error}")
  endif()
endfunction()

expect_rejected("Unknown source" --source replay)
expect_rejected("Unknown argument: --manifest" --manifest unused.json)

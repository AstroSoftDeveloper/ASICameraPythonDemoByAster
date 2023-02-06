pybind11_add_module(ASICamera ASICamera.cpp)

target_link_libraries(ASICamera PRIVATE ASICamera2)

# 设置 VS 中 minicv 项目的调试命令和工作路径
set_target_properties(ASICamera PROPERTIES
    VS_DEBUGGER_COMMAND ${PYTHON_EXECUTABLE}
    VS_DEBUGGER_COMMAND_ARGUMENTS TestFunctions.py
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$(Configuration)")

# 将测试需要的文件，复制到二进制目录中去
#foreach (CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
    #add_custom_command(TARGET ASICamera POST_BUILD
     #   COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_SOURCE_DIR}/ASI_SDK/lib/x64/ASICamera2.dll" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CONFIGURATION_TYPE}/ASICamera2.dll"
      #  COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/TestASICamera.py" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CONFIGURATION_TYPE}/TestASICamera.py")
#endforeach()

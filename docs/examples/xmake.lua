set_project("new_robot_workspace")
set_xmakever("3.0.7")
add_rules("mode.debug", "mode.release")
add_rules("plugin.compile_commands.autoupdate", {outputdir = os.projectdir()})

-- 工作区根目录唯一正式构建入口。
includes("new_robot_code/workspace_target.lua")

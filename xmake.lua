add_rules("mode.debug", "mode.release")
add_requires("serial")
set_toolchains("clang")

option("type")
    set_default("infantry")
    set_showmenu(true)
    set_description("指定要编译的机器人类型")
    set_values("infantry","hero","sentry")
    after_check(function(option)
        option:add("defines", "CONFIG_" .. string.upper(option:value()))
        option:set("basename",option:value())
    end)

target("GKDControl")
    set_kind("binary")
    set_languages("c++23")
    add_files(
        "src/*.cc",
        "src/**/*.cc"
    )
    add_includedirs(
        "include",
        "include/chassis",
        "include/configs",
        "include/device",
        "include/device/referee",
        "include/gimbal",
        "include/utils",
        "include/logger"
    )
    add_packages("serial")
    set_warnings("allextra")

    add_options("type")
    set_basename(get_config("type"))

    if(is_mode("debug")) then 
        add_defines("__DEBUG__")
    end
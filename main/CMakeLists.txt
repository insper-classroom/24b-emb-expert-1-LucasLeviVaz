add_executable(main main.c)

# pull in common dependencies
target_link_libraries(main pico_stdlib hardware_pwm pico_servo hardware_adc freertos)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(main)





# add url via pico_set_program_url
#example_auto_set_url(main)
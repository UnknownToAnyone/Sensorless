################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
F2837xD_common/tools/usb_bulk_example/bulk_libusb.obj: ../F2837xD_common/tools/usb_bulk_example/bulk_libusb.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.6.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="E:/TI/workspace/F2837xD_common/include" --include_path="E:/TI/workspace/F2837xD_common/source" --include_path="E:/TI/workspace/F2837xD_headers/source" --include_path="E:/TI/workspace/F2837xD_headers/include" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.6.LTS/include" --advice:performance=all --define=CPU1 -g --diag_suppress=1311 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/tools/usb_bulk_example/bulk_libusb.d_raw" --obj_directory="F2837xD_common/tools/usb_bulk_example" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/tools/usb_bulk_example/bulk_winusb.obj: ../F2837xD_common/tools/usb_bulk_example/bulk_winusb.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.6.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="E:/TI/workspace/F2837xD_common/include" --include_path="E:/TI/workspace/F2837xD_common/source" --include_path="E:/TI/workspace/F2837xD_headers/source" --include_path="E:/TI/workspace/F2837xD_headers/include" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.6.LTS/include" --advice:performance=all --define=CPU1 -g --diag_suppress=1311 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/tools/usb_bulk_example/bulk_winusb.d_raw" --obj_directory="F2837xD_common/tools/usb_bulk_example" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2837xD_common/tools/usb_bulk_example/usb_bulk_example.obj: ../F2837xD_common/tools/usb_bulk_example/usb_bulk_example.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.6.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="E:/TI/workspace/F2837xD_common/include" --include_path="E:/TI/workspace/F2837xD_common/source" --include_path="E:/TI/workspace/F2837xD_headers/source" --include_path="E:/TI/workspace/F2837xD_headers/include" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.6.LTS/include" --advice:performance=all --define=CPU1 -g --diag_suppress=1311 --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="F2837xD_common/tools/usb_bulk_example/usb_bulk_example.d_raw" --obj_directory="F2837xD_common/tools/usb_bulk_example" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '



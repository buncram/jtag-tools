#### ReRAM0 ############################
sudo python ./jtag_tool.py -f ./RRAM_CP1_power_on_initial_setting_mini_cp.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_istandby.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_inap.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_main_array_verify_mrg0c.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_main_array_verify_mrg0A.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_Strong_write_the_setting_and_codes.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_recall_ifr1.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_check_trim_data_ifr1.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_full_cp_red_vform.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_full_cp_red_training.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_full_cp_main_array_only_vform.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_full_cp_main_array_only_training.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_full_cp_ifr_vform.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_full_cp_ifr_training.tex -b 0
#sudo python ./jtag_tool.py -f ./RRAM_write_ifr_x_0x0_y_0xc_ecc_on_all0.tex -b 0
#### ReRAM1 ############################
sudo python ./jtag_tool.py -f ./RRAM_CP1_power_on_initial_setting_mini_cp.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_istandby.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_inap.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_main_array_verify_mrg0c.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_main_array_verify_mrg0A.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_Strong_write_the_setting_and_codes.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_recall_ifr1.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_check_trim_data_ifr1.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_full_cp_red_vform.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_full_cp_red_training.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_full_cp_main_array_only_vform.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_full_cp_main_array_only_training.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_full_cp_ifr_vform.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_full_cp_ifr_training.tex -b 1
#sudo python ./jtag_tool.py -f ./RRAM_write_ifr_x_0x0_y_0xc_ecc_on.tex -b 1
#### pre-condition RRAM0 and RRAM1 IFR region X=0x0,Y=0x1/0x4/0x6/0xc
sudo python ./jtag_tool.py -f ./RRAM_write_ifr_RISCV_on_x_0x0_y_0x1_0x4_0x6_0xc_ecc_on.tex
#### Return RRAM0/RRAM1 to nap state and Read back from IFR X=0x0, Y=0xC #########
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_inap.tex -b 0
sudo python ./jtag_tool.py -f ./RRAM_mini_cp_inap.tex -b 1
sudo python ./jtag_tool.py -f ./RRAM_read_ifr_x_0x0_y_0xc_ecc_on_after_vform.tex

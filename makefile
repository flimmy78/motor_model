DIR1 = "./pi_reg_speed"
DIR2 = "./pi_reg_cur"


all:
		$(MAKE) -C $(DIR1)
		$(MAKE) -C $(DIR2)

clean:
		$(MAKE) -C $(DIR1) clean
		$(MAKE) -C $(DIR2) clean


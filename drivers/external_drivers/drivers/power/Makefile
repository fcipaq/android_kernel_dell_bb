obj-$(CONFIG_PMIC_CCSM) += pmic_ccsm.o
obj-$(CONFIG_CHARGER_BQ24192)	+= bq24192_charger.o

ifdef CONFIG_BQ24261_CHARGER_EP
	obj-$(CONFIG_BQ24261_CHARGER)	+= bq24261_charger_ep.o
else
	obj-$(CONFIG_BQ24261_CHARGER)	+= bq24261_charger.o
endif



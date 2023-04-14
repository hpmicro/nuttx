############################################################################
# boards/risc-v/hpm6750/hpm6750evk2/src/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

CSRCS  = hpm6750_bringup.c hpm6750_boot.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += hpm6750_appinit.c
endif

ifeq ($(CONFIG_USERLED),y)
  CSRCS += hpm6750_userleds.c
endif

ifeq ($(CONFIG_HPM_I2C_DRV),y)
  CSRCS += hpm6750_i2c.c
endif

ifeq ($(CONFIG_HPM_CAN_DRV),y)
  CSRCS += hpm6750_can.c
endif

ifeq ($(CONFIG_SPI_DRIVER),y)
  CSRCS += hpm6750_spi.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
  CSRCS += hpm6750_gpio.c
endif

ifeq ($(CONFIG_PWM),y)
  CSRCS += hpm6750_pwm.c
endif

include $(TOPDIR)/boards/Board.mk

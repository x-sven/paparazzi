#!/bin/sh
#
# $Id$
#
# SAS Busca_Razor_40cm Busca_Razor_80cm TWOG_CHNI Lerche MJ5 Busca2
for i in SAS Busca_Razor_40cm Busca_Razor_80cm TWOG_CHNI Lerche MJ5 Busca2 HB_MINI
do
	echo "###################### $i #######################"
	(
		make AIRCRAFT=$i clean_ac ap.compile
	) 2>&1 | egrep Total\|Error\|Fehler
done

for i in 1 2 3 4 # defect 5
do
	echo "###################### demo$i #######################"
	(
		make AIRCRAFT=DEMO clean_ac demo$i.compile
	) 2>&1 | egrep Total\|Error\|Fehler
done

# Busca1
# Busca1Razor
# Busca2G
# Busca2GL
# Busca2L
# Busca3
# DEMO
# MJ5

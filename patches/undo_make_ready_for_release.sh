#!/bin/bash


echo "remove application name change patch"
patch -p1 -i ./patches/srslte_emane_application_name.patch -R

echo "remove minimal install patch"
patch -p1 -i ./patches/srslte_emane_minimal_install.patch -R

echo "remove packaging patch"
patch -p1 -i ./patches/srslte_emane_package.patch -R

echo "done"

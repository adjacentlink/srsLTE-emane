#!/bin/bash


echo "apply application name change patch"
patch -p1 -i ./patches/srslte_emane_application_name.patch

echo "apply minimal install patch"
patch -p1 -i ./patches/srslte_emane_minimal_install.patch

echo "apply packaging patch"
patch -p1 -i ./patches/srslte_emane_package.patch

echo "done"

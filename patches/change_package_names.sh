#!/bin/bash

# ./patches/change_package_hames.sh
# git diff > patches/srslte_emane_application_name.patch

sed -i -e 's/srslte_rf_utils /srslte_rf_emane_utils /g' $(find . -name CMakeLists.txt)
sed -i -e 's/srslte_rf /srslte_rf_emane /g' $(find . -name CMakeLists.txt)
sed -i -e 's/srsenb /srsenb-emane /g' $(find . -name CMakeLists.txt)
sed -i -e 's/srsue /srsue-emane /g' $(find . -name CMakeLists.txt)
sed -i -e 's/srsepc /srsepc-emane /g' $(find . -name CMakeLists.txt)
sed -i -e 's/srsmbms /srsmbms-emane /g' $(find . -name CMakeLists.txt)
sed -i -e 's/srslte_rf)/srslte_rf_emane)/g' $(find . -name CMakeLists.txt)

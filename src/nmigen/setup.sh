#!/bin/sh

python3 -m venv .env

. .env/bin/activate

pip install git+https://github.com/nmigen/nmigen.git#egg=nmigen[builtin-yosys]
pip install git+https://github.com/nmigen/nmigen-boards.git
pip install git+https://github.com/nmigen/nmigen-soc.git
pip install git+https://github.com/nmigen/nmigen-stdio.git

#pip install -r pip.require.txt

deactivate

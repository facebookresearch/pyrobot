#!/usr/bin/env bash
# show result in web browser
python run_test.py --out_dir htmlcov/`git rev-parse HEAD`
## not show result in web browser
#python run_test.py --out_dir htmlcov/`git rev-parse HEAD` --nobrowser
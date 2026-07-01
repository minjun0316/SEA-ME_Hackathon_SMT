"""@file conftest.py
@brief 프로젝트 루트를 sys.path에 추가하여 `core`/`sim` 임포트를 보장한다.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

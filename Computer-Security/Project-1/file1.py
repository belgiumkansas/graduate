#!/usr/bin/python
# -*- coding: utf-8 -*-
blob = """
           y��M��mxm����ۨKξ9�7a�9b��b��oF��'S�����[���˻lg���������2�:õ�QAx6�([v���O�MA��il�?]�kyk���/��m�ƶ��W�`W�"""
from hashlib import sha256
print sha256(blob).hexdigest()

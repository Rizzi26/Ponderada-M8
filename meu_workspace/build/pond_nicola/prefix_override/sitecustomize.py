import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rizzi/Documentos/GitHub/ponderada-m8/meu_workspace/install/pond_nicola'

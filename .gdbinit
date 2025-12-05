python
import sys
sys.path.insert(0, 'gdb_printers')
import pose_printer
pose_printer.register_printers(None)
end

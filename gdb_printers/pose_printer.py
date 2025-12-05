import gdb

def _extract_vec3(val):
    arr = val['m_storage']['m_data']['array']
    return (
        float(arr[0]),
        float(arr[1]),
        float(arr[2]),
    )

class PosePrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        pos = _extract_vec3(self.val['position'])
        rot = _extract_vec3(self.val['rotation'])
        return f"Pose(pos={pos}, rot={rot})"


def build_pretty_printer():
    pp = gdb.printing.RegexpCollectionPrettyPrinter("ipc_printers")

    # Match ipc::rigid::Pose<T> for double or any other T
    pp.add_printer(
        "Pose",
        r"^ipc::rigid::Pose<.*>$",
        PosePrinter
    )

    return pp


def register_printers(objfile):
    gdb.printing.register_pretty_printer(objfile, build_pretty_printer())

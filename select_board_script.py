Import('env')
from os.path import join, realpath

# private library flags
for item in env.get("CPPDEFINES", []):
    if isinstance(item, tuple) and item[0] == "LUOSHAL":
        env.Append(CPPPATH=[realpath(item[1])])
        env.Replace(SRC_FILTER=[ "+<%s>" % item[1]])
        break

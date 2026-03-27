# -*- python -*-



def module_main(name, prefix, cname, deps):
    native.genrule(
        name = "{}_main".format(name),
        outs = ["{}_main.cc".format(name)],
        cmd = """cat > $(location {name}_main.cc) << EOF
#include "{prefix}/{name}.h"

#include "base/module_main.h"

extern "C" {{
int main(int argc, char**argv) {{
        return mjmech::base::main<{cname}>(argc, argv);
}}
}}
EOF
        """.format(name=name, cname=cname, prefix=prefix),
    )

    native.cc_binary(
        name = "{}".format(name),
        srcs = [
            "{}_main.cc".format(name),
            name + ".h",
        ],
        deps = deps + [
            "@boost//:filesystem",
            "@boost//:date_time",
            "@org_llvm_libcxx//:libcxx",
        ],
    )

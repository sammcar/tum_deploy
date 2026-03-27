# -*- python -*-



def _dictify(data):
    result = dict([x.split('=', 1) for x in data])
    return result

def _template_file_impl(ctx):
    out = ctx.actions.declare_file(ctx.label.name)

    substitutions = dict(ctx.attr.substitutions)
    substitutions.update(_dictify(ctx.attr.substitution_list))

    ctx.actions.expand_template(
        template = ctx.files.src[0],
        output = out,
        substitutions = substitutions,
        is_executable = ctx.attr.is_executable)
    return [DefaultInfo(
        files = depset([out]),
        data_runfiles = ctx.runfiles(files = [out]),
    )]


template_file = rule(
    attrs = {
        "src": attr.label(allow_files = True),
        "is_executable": attr.bool(default = False),
        "substitutions": attr.string_dict(),
        "substitution_list": attr.string_list(),
    },
    output_to_genfiles = True,
    implementation = _template_file_impl,
)

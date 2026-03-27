# -*- python -*-



load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def github_archive(name, repo, commit, local_override=None,
                   sha256=None, **kwargs):
    """Like 'http_archive', but for github repositories.

    If 'local_override' is set, then reference a local repository at
    the given path instead of github.com.
    """

    if local_override:
        if 'build_file' in kwargs:
            build_file = kwargs['build_file']
            build_path = build_file.package + '/' + build_file.name
            native.new_local_repository(
                name = name,
                build_file = build_path,
                path = local_override,
            )
        else:
            native.local_repository(
                name = name,
                path = local_override,
            )
    else:
        http_archive(
            name = name,
            url = "https://github.com/{repo}/archive/{commit}.zip".format(
                repo=repo, commit=commit),
            strip_prefix = "{}-{}".format(repo.rsplit('/', 1)[-1], commit),
            sha256 = sha256 or "0000000000000000000000000000000000000000000000000000000000000000",
            **kwargs)

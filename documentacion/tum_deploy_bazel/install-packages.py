'''%prog [options]

Install packages necessary for tum.
'''
import optparse
import re
import subprocess
import sys

# Verbosity level:
#  0 - print summaries and all commands which change state ('sudo' commands)
#  1 - print every command executed
g_verbosity = 1

def split(val):
    return [y for y in [x.strip() for x in re.split('\s+', val)]
            if y != '']

def call(*args, **kwargs):
    if g_verbosity >= 1:
        assert 'cwd' not in kwargs, 'not implemented'
        print('call:', args)
        kwargs['stdout'] = sys.stdout
        kwargs['stderr'] = sys.stderr
    kwargs['shell'] = True
    rv = subprocess.call(*args, **kwargs)
    if g_verbosity >= 1:
        print('call result:', rv)
    return rv

def check_call(cmd, **kwargs):
    # always show 'sudo' commands so user knows what he types password for
    is_sudo = kwargs.pop('is_sudo', None)
    # if command generates lots of output we might as well show the command
    # itself
    is_chatty = kwargs.pop('is_chatty', None)
    if g_verbosity >= 1 or is_sudo or is_chatty:
        cmd_v = cmd
        if 'cwd' in kwargs:
            cmd_v = 'cd %s && %s' % (kwargs['cwd'], cmd_v)
        print('check_call:', cmd_v)
        kwargs['stdout'] = sys.stdout
        kwargs['stderr'] = sys.stderr
    kwargs['shell'] = True
    return subprocess.check_call(cmd, **kwargs)

def check_output(*args, **kwargs):
    if g_verbosity >= 1:
        print('check_output:', args)
    kwargs['shell'] = True
    result = subprocess.check_output(*args, **kwargs).strip()
    if g_verbosity >= 1:
        print('check_output_result:')
        if result == '':
            print('', '(empty string)')
        else:
            print(result)
    return result

def main():
    global g_verbosity
    usage, description = __doc__.split('\n\n', 1)
    parser = optparse.OptionParser(usage=usage, description=description)

    parser.add_option('--verbose', '-v', action='count', default=0,
                      help='display additional debugging information')
    parser.add_option('--quiet', '-q', action='count', default=0,
                      help='do not be verbose')
    parser.add_option('--yes', '-y', action='store_true',
                      help='do not ask for confirmation')
    parser.add_option('--system', action='store_true',
                      help='also install system packages')
    parser.add_option('--test', '-t', action='store_true',
                      help='just test if anything needs installing')

    options, args = parser.parse_args()
    g_verbosity += options.verbose - options.quiet

    PKGLIST = split('''
    bison flex gcc g++ curl libc6-dev mesa-common-dev libglu1-mesa-dev
    libtinfo5 libxrender-dev libxi-dev libx11-xcb-dev
    ''')

    ubuntu_release = subprocess.check_output(["lsb_release", "-cs"]).strip()
    if options.verbose:
        print('Ubuntu release:', ubuntu_release)

    if ubuntu_release == b'focal':
        PKGLIST += split('''python-is-python3 libtinfo5''')
    
    # Special handling for Ubuntu 24.04 (Noble) - install libtinfo5 from .deb
    if ubuntu_release == b'noble':
        print('Detected Ubuntu 24.04 (Noble) - installing libtinfo5 from .deb package')
        if not options.test:
            check_call('sudo apt update', is_sudo=True)
            check_call('wget http://security.ubuntu.com/ubuntu/pool/universe/n/ncurses/libtinfo5_6.3-2ubuntu0.1_amd64.deb')
            check_call('sudo apt install %s ./libtinfo5_6.3-2ubuntu0.1_amd64.deb' % 
                      ('-y' if options.yes else ''), is_sudo=True)
        # Remove libtinfo5 from PKGLIST since we're installing it manually
        PKGLIST = [pkg for pkg in PKGLIST if pkg != 'libtinfo5']

    if options.yes or call(
        "apt-get install --dry-run %s |"
        "egrep ' could not be installed|^Conf'" % ' '.join(PKGLIST)) == 0:
        print()
        print('Need to install some packages')
        if options.test:
            sys.exit(1)
        cmd = ('sudo apt-get install %s %s' % (
            ('-y' if options.yes else ''), ' '.join(PKGLIST)))
        check_call(cmd, is_sudo=True)
    else:
        print('All packages up to date')

if __name__ == '__main__':
    main()
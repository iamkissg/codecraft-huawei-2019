def strip_parenthesis(line: str):
    return line.lstrip('(').rstrip(')')


def read_file_and_yield_info(fname):
    with open(fname, ) as fin:
        while True:
            line = fin.readline().strip()
            if line == '':
                break
            if line.startswith('#'):
                continue
            info = [item.strip() for item in strip_parenthesis(line).split(',')]
            yield info
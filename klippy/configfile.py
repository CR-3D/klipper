# Code for reading and writing the Klipper config file
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys, os, glob, re, time, logging, configparser, io, ast, json

error = configparser.Error


######################################################################
# Config section parsing helper
######################################################################

class sentinel:
    pass

class ConfigWrapper:
    error = configparser.Error
    def __init__(self, printer, fileconfig, access_tracking, section):
        self.printer = printer
        self.fileconfig = fileconfig
        self.access_tracking = access_tracking
        self.section = section
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.section
    def _get_wrapper(self, parser, option, default, minval=None, maxval=None,
                     above=None, below=None, note_valid=True):
        if not self.fileconfig.has_option(self.section, option):
            if default is not sentinel:
                if note_valid and default is not None:
                    acc_id = (self.section.lower(), option.lower())
                    self.access_tracking[acc_id] = default
                return default
            raise error("Option '%s' in section '%s' must be specified"
                        % (option, self.section))
        try:
            v = parser(self.section, option)
        except self.error as e:
            raise
        except:
            raise error("Unable to parse option '%s' in section '%s'"
                        % (option, self.section))
        if note_valid:
            self.access_tracking[(self.section.lower(), option.lower())] = v
        if minval is not None and v < minval:
            raise error("Option '%s' in section '%s' must have minimum of %s"
                        % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise error("Option '%s' in section '%s' must have maximum of %s"
                        % (option, self.section, maxval))
        if above is not None and v <= above:
            raise error("Option '%s' in section '%s' must be above %s"
                        % (option, self.section, above))
        if below is not None and v >= below:
            raise self.error("Option '%s' in section '%s' must be below %s"
                             % (option, self.section, below))
        return v
    def get(self, option, default=sentinel, note_valid=True):
        return self._get_wrapper(self.fileconfig.get, option, default,
                                 note_valid=note_valid)
    def getint(self, option, default=sentinel, minval=None, maxval=None,
               note_valid=True):
        return self._get_wrapper(self.fileconfig.getint, option, default,
                                 minval, maxval, note_valid=note_valid)
    def getfloat(self, option, default=sentinel, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        return self._get_wrapper(self.fileconfig.getfloat, option, default,
                                 minval, maxval, above, below,
                                 note_valid=note_valid)
    def getboolean(self, option, default=sentinel, note_valid=True):
        return self._get_wrapper(self.fileconfig.getboolean, option, default,
                                 note_valid=note_valid)
    def getchoice(self, option, choices, default=sentinel, note_valid=True):
        if type(choices) == type([]):
            choices = {i: i for i in choices}
        if choices and type(list(choices.keys())[0]) == int:
            c = self.getint(option, default, note_valid=note_valid)
        else:
            c = self.get(option, default, note_valid=note_valid)
        if c not in choices:
            raise error("Choice '%s' for option '%s' in section '%s'"
                        " is not a valid choice" % (c, option, self.section))
        return choices[c]
    def getlists(self, option, default=sentinel, seps=(',',), count=None,
                 parser=str, note_valid=True):
        def lparser(value, pos):
            if len(value.strip()) == 0:
                # Return an empty list instead of [''] for empty string
                parts = []
            else:
                parts = [p.strip() for p in value.split(seps[pos])]
            if pos:
                # Nested list
                return tuple([lparser(p, pos - 1) for p in parts if p])
            res = [parser(p) for p in parts]
            if count is not None and len(res) != count:
                raise error("Option '%s' in section '%s' must have %d elements"
                            % (option, self.section, count))
            return tuple(res)
        def fcparser(section, option):
            return lparser(self.fileconfig.get(section, option), len(seps) - 1)
        return self._get_wrapper(fcparser, option, default,
                                 note_valid=note_valid)
    def getlist(self, option, default=sentinel, sep=',', count=None,
                note_valid=True):
        return self.getlists(option, default, seps=(sep,), count=count,
                             parser=str, note_valid=note_valid)
    def getintlist(self, option, default=sentinel, sep=',', count=None,
                   note_valid=True):
        return self.getlists(option, default, seps=(sep,), count=count,
                             parser=int, note_valid=note_valid)
    def getfloatlist(self, option, default=sentinel, sep=',', count=None,
                     note_valid=True):
        return self.getlists(option, default, seps=(sep,), count=count,
                             parser=float, note_valid=note_valid)
    def getsection(self, section):
        return ConfigWrapper(self.printer, self.fileconfig,
                             self.access_tracking, section)
    def has_section(self, section):
        return self.fileconfig.has_section(section)
    def get_prefix_sections(self, prefix):
        return [self.getsection(s) for s in self.fileconfig.sections()
                if s.startswith(prefix)]
    def get_prefix_options(self, prefix):
        return [o for o in self.fileconfig.options(self.section)
                if o.startswith(prefix)]
    def deprecate(self, option, value=None):
        if not self.fileconfig.has_option(self.section, option):
            return
        pconfig = self.printer.lookup_object("configfile")
        pconfig.deprecate(self.section, option, value)


######################################################################
# Config file parsing (with include file support)
######################################################################

# Section reserved for user defined values referenced by variables and
# include conditions. It has no runtime object, so the unused-options
# check skips it (as well as sections loaded with [include_json]).
CONSTANTS_SECTION = 'constants'

# Conditional include of the form "[include if:${expression} path.cfg]"
_CONDITIONAL_INCLUDE_RE = re.compile(r"if:\$\{(.+?)\}\s+(.*)")

# Variable reference of the form "${[section.]option[:default]}". A
# reference preceded by a backslash ("\${...}") is left as literal text.
_VARIABLE_RE = re.compile(
    r"(?<!\\)\$\{"
    r"(?:(?P<section>[^.:${}]+)\.)?"
    r"(?P<option>[^${}:]+)"
    r"(?::(?P<default>[^${}]*))?"
    r"\}")

# Arithmetic in config values: numbers, + - * /, parentheses and the
# functions below. Anything else is left untouched.
_ARITHMETIC_CHARS_RE = re.compile(r"^[\w\s.+\-*/(),]+$")
_ARITHMETIC_FUNCS = {'min': min, 'max': max, 'abs': abs, 'round': round}
_ARITHMETIC_BINOPS = {
    ast.Add: lambda a, b: a + b, ast.Sub: lambda a, b: a - b,
    ast.Mult: lambda a, b: a * b, ast.Div: lambda a, b: a / b,
}
_ARITHMETIC_UNARYOPS = {ast.UAdd: lambda a: +a, ast.USub: lambda a: -a}
_ARITHMETIC_NUM_NODE = getattr(ast, 'Constant', None) or ast.Num

class _NotArithmetic(Exception):
    pass

def _eval_arithmetic_node(node):
    if isinstance(node, ast.Expression):
        return _eval_arithmetic_node(node.body)
    if isinstance(node, _ARITHMETIC_NUM_NODE):
        num = getattr(node, 'value', getattr(node, 'n', None))
        if isinstance(num, (int, float)) and not isinstance(num, bool):
            return num
        raise _NotArithmetic()
    if isinstance(node, ast.BinOp) and type(node.op) in _ARITHMETIC_BINOPS:
        return _ARITHMETIC_BINOPS[type(node.op)](
            _eval_arithmetic_node(node.left), _eval_arithmetic_node(node.right))
    if (isinstance(node, ast.UnaryOp)
        and type(node.op) in _ARITHMETIC_UNARYOPS):
        return _ARITHMETIC_UNARYOPS[type(node.op)](
            _eval_arithmetic_node(node.operand))
    if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        and node.func.id in _ARITHMETIC_FUNCS and not node.keywords):
        args = [_eval_arithmetic_node(a) for a in node.args]
        return _ARITHMETIC_FUNCS[node.func.id](*args)
    raise _NotArithmetic()

def _evaluate_arithmetic_item(value):
    """Returns the result of value as a string if it is an arithmetic
    expression (eg, "250 - 2 * 5" or "max(10, 20) / 2"), otherwise value
    unchanged. Plain numbers are never rewritten."""
    if not value.strip() or not _ARITHMETIC_CHARS_RE.match(value):
        return value
    try:
        tree = ast.parse(value.strip(), mode='eval')
    except SyntaxError:
        return value
    if not any(isinstance(n, (ast.BinOp, ast.Call)) for n in ast.walk(tree)):
        return value
    try:
        result = _eval_arithmetic_node(tree)
    except _NotArithmetic:
        return value
    except (ArithmeticError, TypeError, ValueError) as e:
        raise error("Unable to evaluate expression '%s': %s"
                    % (value.strip(), e))
    if isinstance(result, float) and result.is_integer():
        result = int(result)
    if isinstance(result, float):
        result = "%.12g" % (result,)
    # Keep surrounding whitespace so list and multi-line layout is preserved
    lead = value[:len(value) - len(value.lstrip())]
    trail = value[len(value.rstrip()):]
    return lead + str(result) + trail

def _evaluate_arithmetic(value):
    """Evaluates arithmetic expressions in a config value. Each line and
    each comma separated list element (outside of parentheses) is
    evaluated on its own, so "250 - 10, 250 - 10" becomes "240, 240"."""
    lines = []
    for line in value.split('\n'):
        items = []
        depth = start = 0
        for i, c in enumerate(line):
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
            elif c == ',' and depth == 0:
                items.append(line[start:i])
                start = i + 1
        items.append(line[start:])
        lines.append(','.join([_evaluate_arithmetic_item(i) for i in items]))
    return '\n'.join(lines)

class ConfigVariableResolver:
    """Resolves "${section.option}" references against a fileconfig. A
    reference without a section refers to an option of the same section,
    or of the [constants] section if the same section does not have it.
    If the referenced option does not exist, the default after the colon
    is used (if given). Afterwards arithmetic expressions are evaluated."""
    def __init__(self, fileconfig):
        self.fileconfig = fileconfig
        self.cache = {}
        self.in_progress = []
    def resolve(self, section, option):
        key = (section, option)
        if key in self.cache:
            return self.cache[key]
        if key in self.in_progress:
            chain = self.in_progress[self.in_progress.index(key):] + [key]
            raise error("Circular variable reference: %s" % (
                " -> ".join(["%s.%s" % k for k in chain]),))
        self.in_progress.append(key)
        try:
            value = self.fileconfig.get(section, option)
            value = self._substitute(
                value, section, "Option '%s' in section '%s'"
                % (option, section))
            try:
                value = _evaluate_arithmetic(value)
            except error as e:
                raise error("Option '%s' in section '%s': %s"
                            % (option, section, e))
        finally:
            self.in_progress.pop()
        self.cache[key] = value
        return value
    def _substitute(self, value, section, owner):
        def lookup(m):
            ref_option = m.group('option').strip()
            if m.group('section'):
                candidates = [m.group('section').strip()]
            else:
                candidates = [section, CONSTANTS_SECTION]
            for ref_section in candidates:
                if (self.fileconfig.has_section(ref_section)
                    and self.fileconfig.has_option(ref_section, ref_option)):
                    return self.resolve(
                        ref_section, self.fileconfig.optionxform(ref_option))
            if m.group('default') is not None:
                return m.group('default')
            raise error("%s references undefined option '%s.%s'"
                        % (owner, candidates[0], ref_option))
        return _VARIABLE_RE.sub(lookup, value)
    def resolve_include_path(self, path):
        # Bare references in include paths refer to [constants]
        value = self._substitute(path, CONSTANTS_SECTION,
                                 "Include '%s'" % (path,))
        return value.replace('\\${', '${')
    def resolve_all(self):
        for section in self.fileconfig.sections():
            for option in self.fileconfig.options(section):
                orig = self.fileconfig.get(section, option)
                value = self.resolve(section, option).replace('\\${', '${')
                if value.strip() == 'None' and _VARIABLE_RE.search(orig):
                    # A reference resolving to "None" removes the option,
                    # so that eg "${constants.x:None}" makes it optional
                    self.fileconfig.remove_option(section, option)
                elif value != orig:
                    self.fileconfig.set(section, option, value)

# JSON data include of the form "[include_json [name:] path.json]"
_JSON_INCLUDE_RE = re.compile(
    r"^(?:(?P<name>[A-Za-z_]\w*)\s*:\s*)?(?P<path>.+)$")

def _json_value_to_str(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "None"
    return str(value)

def _flatten_json(data, prefix=""):
    """Flattens nested JSON objects into "a.b.c" keys. Lists are stored
    as a comma separated list under their key and each element under
    "key.<index>"."""
    items = []
    if isinstance(data, dict):
        for key, value in data.items():
            items.extend(_flatten_json(value, prefix + str(key) + "."))
        return items
    key = prefix[:-1]
    if isinstance(data, list):
        if all(not isinstance(v, (dict, list)) for v in data):
            items.append((key, ", ".join([_json_value_to_str(v)
                                          for v in data])))
        for i, value in enumerate(data):
            items.extend(_flatten_json(value, "%s.%d." % (key, i)))
        return items
    items.append((key, _json_value_to_str(data)))
    return items

class ConfigNamespace:
    """Exposes the options of a config section as attributes so that a
    conditional include expression like "constants.has_probe" can be
    evaluated as Python code."""
    def __init__(self, data):
        for key, value in data.items():
            setattr(self, key, value)
    def __getitem__(self, item):
        return getattr(self, item)
    def __repr__(self):
        return str(self.__dict__)

def _convert_condition_value(value):
    lvalue = value.lower()
    if lvalue == "true":
        return True
    if lvalue == "false":
        return False
    try:
        if value.isdigit():
            return int(value)
        if "." in value:
            return float(value)
    except ValueError:
        pass
    return value

class ConfigFileReader:
    def read_config_file(self, filename):
        try:
            f = open(filename, 'r')
            data = f.read()
            f.close()
        except:
            msg = "Unable to open config file %s" % (filename,)
            logging.exception(msg)
            raise error(msg)
        return data.replace('\r\n', '\n')
    def build_config_string(self, fileconfig):
        sfile = io.StringIO()
        fileconfig.write(sfile)
        return sfile.getvalue().strip()
    def append_fileconfig(self, fileconfig, data, filename):
        if not data:
            return
        # Strip trailing comments
        lines = data.split('\n')
        for i, line in enumerate(lines):
            pos = line.find('#')
            if pos >= 0:
                lines[i] = line[:pos]
        sbuffer = io.StringIO('\n'.join(lines))
        if sys.version_info.major >= 3:
            fileconfig.read_file(sbuffer, filename)
        else:
            fileconfig.readfp(sbuffer, filename)
    def _create_fileconfig(self):
        if sys.version_info.major >= 3:
            fileconfig = configparser.RawConfigParser(
                strict=False, inline_comment_prefixes=(';', '#'))
        else:
            fileconfig = configparser.RawConfigParser()
        # Sections that only hold data (no runtime object)
        fileconfig.data_sections = set([CONSTANTS_SECTION])
        return fileconfig
    def build_fileconfig(self, data, filename):
        fileconfig = self._create_fileconfig()
        self.append_fileconfig(fileconfig, data, filename)
        return fileconfig
    def _check_include_condition(self, expression, fileconfig):
        resolver = ConfigVariableResolver(fileconfig)
        def get_value(section, option):
            try:
                return resolver.resolve(section, option)
            except error:
                return fileconfig.get(section, option)
        def build_namespace(section):
            # Dotted options (from JSON) become nested namespaces, so that
            # "cfg.probe.enabled" works as attribute access
            tree = {}
            for option in fileconfig.options(section):
                value = _convert_condition_value(get_value(section, option))
                node = tree
                parts = option.split('.')
                for part in parts[:-1]:
                    node = node.setdefault(part, {})
                    if not isinstance(node, dict):
                        break
                else:
                    node.setdefault(parts[-1], value)
            def to_namespace(node):
                return ConfigNamespace(
                    {k: to_namespace(v) if isinstance(v, dict) else v
                     for k, v in node.items()})
            return to_namespace(tree)
        context = {section: build_namespace(section)
                   for section in fileconfig.sections()}
        try:
            return eval(expression, {"__builtins__": {}}, context)
        except Exception as e:
            logging.warning("Failed to evaluate include condition '%s': %s",
                            expression, e)
            return False
    def _resolve_include(self, source_filename, include_spec, fileconfig,
                         visited):
        include_spec = include_spec.strip()
        condition_match = _CONDITIONAL_INCLUDE_RE.match(include_spec)
        if condition_match:
            expression, include_spec = condition_match.groups()
            if not self._check_include_condition(expression, fileconfig):
                logging.info("Include condition '%s' not met, skipping %s",
                             expression, include_spec)
                return []
        if _VARIABLE_RE.search(include_spec):
            include_spec = ConfigVariableResolver(
                fileconfig).resolve_include_path(include_spec)
        dirname = os.path.dirname(source_filename)
        include_spec = include_spec.strip()
        include_glob = os.path.join(dirname, include_spec)
        include_filenames = glob.glob(include_glob)
        if not include_filenames and not glob.has_magic(include_glob):
            # Empty set is OK if wildcard but not for direct file reference
            raise error("Include file '%s' does not exist" % (include_glob,))
        include_filenames.sort()
        for include_filename in include_filenames:
            include_data = self.read_config_file(include_filename)
            self._parse_config(include_data, include_filename, fileconfig,
                               visited)
        return include_filenames
    def _resolve_json_include(self, source_filename, include_spec,
                              fileconfig):
        mo = _JSON_INCLUDE_RE.match(include_spec.strip())
        if mo is None:
            raise error("Invalid include_json '%s'" % (include_spec,))
        path = mo.group('path').strip()
        if _VARIABLE_RE.search(path):
            path = ConfigVariableResolver(fileconfig).resolve_include_path(
                path)
        json_filename = os.path.join(os.path.dirname(source_filename), path)
        name = mo.group('name')
        if name is None:
            base = os.path.splitext(os.path.basename(json_filename))[0]
            name = re.sub(r"\W", "_", base)
        if (fileconfig.has_section(name)
            and name.lower() not in fileconfig.data_sections):
            raise error("include_json section name '%s' conflicts with"
                        " an existing config section" % (name,))
        try:
            with open(json_filename, 'r') as f:
                data = json.load(f)
        except (IOError, OSError) as e:
            raise error("Unable to open JSON file '%s': %s"
                        % (json_filename, e.strerror or e))
        except ValueError as e:
            raise error("Unable to parse JSON file '%s': %s"
                        % (json_filename, e))
        if not isinstance(data, dict):
            raise error("JSON file '%s' must contain an object"
                        % (json_filename,))
        if not fileconfig.has_section(name):
            fileconfig.add_section(name)
        fileconfig.data_sections.add(name.lower())
        for option, value in _flatten_json(data):
            fileconfig.set(name, option, value)
    def _parse_config(self, data, filename, fileconfig, visited):
        path = os.path.abspath(filename)
        if path in visited:
            raise error("Recursive include of config file '%s'" % (filename))
        visited.add(path)
        lines = data.split('\n')
        # Buffer lines between includes and parse as a unit so that overrides
        # in includes apply linearly as they do within a single file
        buf = []
        for line in lines:
            # Strip trailing comment
            pos = line.find('#')
            if pos >= 0:
                line = line[:pos]
            # Process include or buffer line
            mo = configparser.RawConfigParser.SECTCRE.match(line)
            header = mo and mo.group('header')
            if header and header.startswith('include_json '):
                self.append_fileconfig(fileconfig, '\n'.join(buf), filename)
                del buf[:]
                self._resolve_json_include(filename, header[13:], fileconfig)
            elif header and header.startswith('include '):
                self.append_fileconfig(fileconfig, '\n'.join(buf), filename)
                del buf[:]
                include_spec = header[8:].strip()
                self._resolve_include(filename, include_spec, fileconfig,
                                      visited)
            else:
                buf.append(line)
        self.append_fileconfig(fileconfig, '\n'.join(buf), filename)
        visited.remove(path)
    def build_fileconfig_with_includes(self, data, filename):
        fileconfig = self._create_fileconfig()
        self._parse_config(data, filename, fileconfig, set())
        ConfigVariableResolver(fileconfig).resolve_all()
        return fileconfig


######################################################################
# Config auto save helper
######################################################################

AUTOSAVE_HEADER = """
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
"""

class ConfigAutoSave:
    def __init__(self, printer):
        self.printer = printer
        self.fileconfig = None
        self.status_save_pending = {}
        self.save_config_pending = False
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("SAVE_CONFIG", self.cmd_SAVE_CONFIG,
                               desc=self.cmd_SAVE_CONFIG_help)
    def _find_autosave_data(self, data):
        regular_data = data
        autosave_data = ""
        pos = data.find(AUTOSAVE_HEADER)
        if pos >= 0:
            regular_data = data[:pos]
            autosave_data = data[pos + len(AUTOSAVE_HEADER):].strip()
        # Check for errors and strip line prefixes
        if "\n#*# " in regular_data or autosave_data.find(AUTOSAVE_HEADER) >= 0:
            logging.warning("Can't read autosave from config file"
                            " - autosave state corrupted")
            return data, ""
        out = [""]
        for line in autosave_data.split('\n'):
            if ((not line.startswith("#*#")
                 or (len(line) >= 4 and not line.startswith("#*# ")))
                and autosave_data):
                logging.warning("Can't read autosave from config file"
                                " - modifications after header")
                return data, ""
            out.append(line[4:])
        out.append("")
        return regular_data, "\n".join(out)
    comment_r = re.compile('[#;].*$')
    value_r = re.compile('[^A-Za-z0-9_].*$')
    def _strip_duplicates(self, data, fileconfig):
        # Comment out fields in 'data' that are defined in 'config'
        lines = data.split('\n')
        section = None
        is_dup_field = False
        for lineno, line in enumerate(lines):
            pruned_line = self.comment_r.sub('', line).rstrip()
            if not pruned_line:
                continue
            if pruned_line[0].isspace():
                if is_dup_field:
                    lines[lineno] = '#' + lines[lineno]
                continue
            is_dup_field = False
            if pruned_line[0] == '[':
                section = pruned_line[1:-1].strip()
                continue
            field = self.value_r.sub('', pruned_line)
            if fileconfig.has_option(section, field):
                is_dup_field = True
                lines[lineno] = '#' + lines[lineno]
        return "\n".join(lines)
    def load_main_config(self):
        filename = self.printer.get_start_args()['config_file']
        cfgrdr = ConfigFileReader()
        data = cfgrdr.read_config_file(filename)
        regular_data, autosave_data = self._find_autosave_data(data)
        regular_fileconfig = cfgrdr.build_fileconfig_with_includes(
            regular_data, filename)
        autosave_data = self._strip_duplicates(autosave_data,
                                               regular_fileconfig)
        self.fileconfig = cfgrdr.build_fileconfig(autosave_data, filename)
        cfgrdr.append_fileconfig(regular_fileconfig,
                                 autosave_data, '*AUTOSAVE*')
        return regular_fileconfig, self.fileconfig
    def get_status(self, eventtime):
        return {'save_config_pending': self.save_config_pending,
                'save_config_pending_items': self.status_save_pending}
    def set(self, section, option, value):
        if not self.fileconfig.has_section(section):
            self.fileconfig.add_section(section)
        svalue = str(value)
        self.fileconfig.set(section, option, svalue)
        pending = dict(self.status_save_pending)
        if not section in pending or pending[section] is None:
            pending[section] = {}
        else:
            pending[section] = dict(pending[section])
        pending[section][option] = svalue
        self.status_save_pending = pending
        self.save_config_pending = True
        logging.info("save_config: set [%s] %s = %s", section, option, svalue)
    def remove_section(self, section):
        if self.fileconfig.has_section(section):
            self.fileconfig.remove_section(section)
            pending = dict(self.status_save_pending)
            pending[section] = None
            self.status_save_pending = pending
            self.save_config_pending = True
        elif (section in self.status_save_pending and
              self.status_save_pending[section] is not None):
            pending = dict(self.status_save_pending)
            del pending[section]
            self.status_save_pending = pending
            self.save_config_pending = True
    def _disallow_include_conflicts(self, regular_fileconfig):
        for section in self.fileconfig.sections():
            for option in self.fileconfig.options(section):
                if regular_fileconfig.has_option(section, option):
                    msg = ("SAVE_CONFIG section '%s' option '%s' conflicts "
                           "with included value" % (section, option))
                    raise self.printer.command_error(msg)
    cmd_SAVE_CONFIG_help = "Overwrite config file and restart"
    def cmd_SAVE_CONFIG(self, gcmd):
        if not self.fileconfig.sections():
            return
        # Create string containing autosave data
        cfgrdr = ConfigFileReader()
        autosave_data = cfgrdr.build_config_string(self.fileconfig)
        lines = [('#*# ' + l).strip()
                 for l in autosave_data.split('\n')]
        lines.insert(0, "\n" + AUTOSAVE_HEADER.rstrip())
        lines.append("")
        autosave_data = '\n'.join(lines)
        # Read in and validate current config file
        cfgname = self.printer.get_start_args()['config_file']
        try:
            data = cfgrdr.read_config_file(cfgname)
        except error as e:
            msg = "Unable to read existing config on SAVE_CONFIG"
            logging.exception(msg)
            raise gcmd.error(msg)
        regular_data, old_autosave_data = self._find_autosave_data(data)
        regular_data = self._strip_duplicates(regular_data, self.fileconfig)
        data = regular_data.rstrip() + autosave_data
        new_regular_data, new_autosave_data = self._find_autosave_data(data)
        if not new_autosave_data:
            raise gcmd.error(
                "Existing config autosave is corrupted."
                " Can't complete SAVE_CONFIG")
        try:
            regular_fileconfig = cfgrdr.build_fileconfig_with_includes(
                new_regular_data, cfgname)
        except error as e:
            msg = "Unable to parse existing config on SAVE_CONFIG"
            logging.exception(msg)
            raise gcmd.error(msg)
        self._disallow_include_conflicts(regular_fileconfig)
        # Determine filenames
        datestr = time.strftime("-%Y%m%d_%H%M%S")
        backup_name = cfgname + datestr
        temp_name = cfgname + "_autosave"
        if cfgname.endswith(".cfg"):
            backup_name = cfgname[:-4] + datestr + ".cfg"
            temp_name = cfgname[:-4] + "_autosave.cfg"
        # Create new config file with temporary name and swap with main config
        logging.info("SAVE_CONFIG to '%s' (backup in '%s')",
                     cfgname, backup_name)
        try:
            f = open(temp_name, 'w')
            f.write(data)
            f.close()
            os.rename(cfgname, backup_name)
            os.rename(temp_name, cfgname)
        except:
            msg = "Unable to write config file during SAVE_CONFIG"
            logging.exception(msg)
            raise gcmd.error(msg)
        # Request a restart
        gcode = self.printer.lookup_object('gcode')
        gcode.request_restart('restart')


######################################################################
# Config validation (check for undefined options)
######################################################################

class ConfigValidate:
    def __init__(self, printer):
        self.printer = printer
        self.status_settings = {}
        self.access_tracking = {}
        self.autosave_options = {}
    def start_access_tracking(self, autosave_fileconfig):
        # Note autosave options for use during undefined options check
        self.autosave_options = {}
        for section in autosave_fileconfig.sections():
            for option in autosave_fileconfig.options(section):
                self.autosave_options[(section.lower(), option.lower())] = 1
        self.access_tracking = {}
        return self.access_tracking
    def check_unused(self, fileconfig):
        # Don't warn on fields set in autosave segment
        access_tracking = dict(self.access_tracking)
        access_tracking.update(self.autosave_options)
        # Note locally used sections
        valid_sections = { s: 1 for s, o in self.printer.lookup_objects() }
        valid_sections.update({ s: 1 for s, o in access_tracking })
        # Validate that there are no undefined parameters in the config file
        for section_name in fileconfig.sections():
            section = section_name.lower()
            if section in getattr(fileconfig, 'data_sections', ()):
                continue
            if section not in valid_sections:
                raise error("Section '%s' is not a valid config section"
                            % (section,))
            for option in fileconfig.options(section_name):
                option = option.lower()
                if (section, option) not in access_tracking:
                    raise error("Option '%s' is not valid in section '%s'"
                                % (option, section))
        # Setup get_status()
        self._build_status_settings()
        # Clear tracking state
        self.access_tracking.clear()
        self.autosave_options.clear()
    def _build_status_settings(self):
        self.status_settings = {}
        for (section, option), value in self.access_tracking.items():
            self.status_settings.setdefault(section, {})[option] = value
    def get_status(self, eventtime):
        return {'settings': self.status_settings}


######################################################################
# Main printer config tracking
######################################################################

class PrinterConfig:
    def __init__(self, printer):
        self.printer = printer
        self.autosave = ConfigAutoSave(printer)
        self.validate = ConfigValidate(printer)
        self.deprecated = {}
        self.status_raw_config = {}
        self.status_warnings = []
    def get_printer(self):
        return self.printer
    def read_config(self, filename):
        cfgrdr = ConfigFileReader()
        data = cfgrdr.read_config_file(filename)
        fileconfig = cfgrdr.build_fileconfig(data, filename)
        return ConfigWrapper(self.printer, fileconfig, {}, 'printer')
    def read_main_config(self):
        fileconfig, autosave_fileconfig = self.autosave.load_main_config()
        access_tracking = self.validate.start_access_tracking(
            autosave_fileconfig)
        config = ConfigWrapper(self.printer, fileconfig,
                               access_tracking, 'printer')
        self._build_status_config(config)
        return config
    def log_config(self, config):
        cfgrdr = ConfigFileReader()
        lines = ["===== Config file =====",
                 cfgrdr.build_config_string(config.fileconfig),
                 "======================="]
        self.printer.set_rollover_info("config", "\n".join(lines))
    def check_unused_options(self, config):
        self.validate.check_unused(config.fileconfig)
    # Deprecation warnings
    def _add_deprecated(self, data):
        key = tuple(list(data.items()))
        if key in self.deprecated:
            return False
        self.deprecated[key] = True
        self.status_warnings = self.status_warnings + [data]
        return True
    def runtime_warning(self, msg):
        res = {'type': 'runtime_warning', 'message': msg}
        did_add = self._add_deprecated(res)
        if did_add:
            logging.warning(msg)
    def deprecate(self, section, option, value=None, msg=None):
        if value is None:
            res = {'type': 'deprecated_option'}
            defmsg = ("Option '%s' in section '%s' is deprecated."
                   % (option, section))
        else:
            res = {'type': 'deprecated_value', 'value': value}
            defmsg = ("Value '%s' in option '%s' in section '%s' is deprecated."
                      % (value, option, section))
        if msg is None:
            msg = defmsg
        res['message'] = msg
        res['section'] = section
        res['option'] = option
        self._add_deprecated(res)
    def deprecate_gcode(self, cmd, param=None, value=None, msg=None):
        if param is None:
            defmsg = "Command '%s' is deprecated." % (cmd,)
        elif value is None:
            defmsg = ("Parameter '%s' in command '%s' is deprecated."
                      % (param, cmd))
        else:
            defmsg = ("Value '%s=%s' in command '%s' is deprecated."
                      % (param, value, cmd))
        if msg is None:
            msg = defmsg
        res = {'type': 'deprecated_gcode', 'message': msg,
               'command': cmd, 'parameter': param, 'value': str(value)}
        self._add_deprecated(res)
    def deprecate_mcu_code(self, mcu, feature, msg=None):
        mcu_name = mcu.get_name()
        if msg is None:
            vhost = self.printer.start_args['software_version']
            vmcu = mcu.get_status()['mcu_version']
            msg = ("MCU '%s' has deprecated code (it is missing feature '%s')."
                   " Recompiling and flashing is recommended (MCU version '%s',"
                   " host version '%s')." % (mcu_name, feature, vmcu, vhost))
        res = {'type': 'deprecated_mcu_code', 'message': msg,
               'mcu': mcu_name, 'feature': feature}
        self._add_deprecated(res)
    # Status reporting
    def _build_status_config(self, config):
        self.status_raw_config = {}
        for section in config.get_prefix_sections(''):
            self.status_raw_config[section.get_name()] = section_status = {}
            for option in section.get_prefix_options(''):
                section_status[option] = section.get(option, note_valid=False)
    def get_status(self, eventtime):
        status = {'config': self.status_raw_config,
                  'warnings': self.status_warnings}
        status.update(self.autosave.get_status(eventtime))
        status.update(self.validate.get_status(eventtime))
        return status
    # Autosave functions
    def set(self, section, option, value):
        self.autosave.set(section, option, value)
    def remove_section(self, section):
        self.autosave.remove_section(section)

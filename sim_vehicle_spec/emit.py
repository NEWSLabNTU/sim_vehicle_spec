"""How generated files are written, the same in every repository that generates from a spec:
the header, YAML with rounded floats, a value as the spec wrote it, comments."""

HEADER = ("Generated from {spec} (spec_hash {hash}) by {tool}: do not edit; "
          "change the spec and regenerate.")


def header(spec, tool, rel=None):
    """The header line every generated file starts with: the spec, its hash, the tool."""
    from .spec import spec_hash

    return HEADER.format(spec=rel or spec.path.name, hash=spec_hash(spec), tool=tool)


def num(v):
    if isinstance(v, bool) or isinstance(v, int):
        return v
    if isinstance(v, float):
        r = round(v, 9)
        return 0.0 if r == 0 else r
    return v


def rounded(v):
    if isinstance(v, dict):
        return {k: rounded(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [rounded(x) for x in v]
    return num(v)


def fmt(v):
    """A value from the spec as written (no rounding: a calibration keeps its digits)."""
    if isinstance(v, bool):
        return "true" if v else "false"
    return repr(v)


def comment(text, mark="#"):
    return "".join(f"{mark} {line}".rstrip() + "\n" for line in text.splitlines())


def yaml_text(doc):
    import yaml

    return yaml.safe_dump(rounded(doc), sort_keys=False, default_flow_style=None, width=100)


def stale(outputs, root):
    """The paths of `outputs` ({path: text}, relative to `root`) whose file differs or is missing."""
    from pathlib import Path

    root = Path(root)
    return [p for p, text in outputs.items()
            if not (root / p).is_file() or (root / p).read_text() != text]


def write(outputs, root):
    from pathlib import Path

    root = Path(root)
    written = []
    for p, text in outputs.items():
        f = root / p
        if f.is_file() and f.read_text() == text:
            continue
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_text(text)
        written.append(p)
    return written

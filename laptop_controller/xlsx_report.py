"""
xlsx_report.py: run report as .xlsx with load-vs-time charts, standard library only.

Sheets: Run (summary + log + charts), Chart lines, Scaled load (full-size values
resampled to a constant step). Written strictly enough for Excel: schema order,
shared strings, numCache on every series, bad XML chars and NaN/inf removed.
"""

from __future__ import annotations

import math
import re
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence
from xml.sax.saxutils import escape as _xml_escape

NS_MAIN = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
NS_R = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
NS_PKG_R = "http://schemas.openxmlformats.org/package/2006/relationships"
NS_A = "http://schemas.openxmlformats.org/drawingml/2006/main"
NS_C = "http://schemas.openxmlformats.org/drawingml/2006/chart"
NS_XDR = "http://schemas.openxmlformats.org/drawingml/2006/spreadsheetDrawing"
REL = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
DECL = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'

# One colour per bolt
BOLT_COLOURS = ["1F77B4", "FF7F0E", "2CA02C", "9467BD", "8C564B", "E377C2",
                "17BECF", "BCBD22", "7F7F7F", "AEC7E8", "FFBB78", "98DF8A"]

# Characters not allowed in XML 1.0
_BAD_XML = re.compile("[\x00-\x08\x0b\x0c\x0e-\x1f￾￿]")


def escape(text) -> str:
    return _xml_escape(_BAD_XML.sub("", str(text)), {'"': "&quot;"})


def _col(i: int) -> str:
    """0 -> A, 25 -> Z, 26 -> AA."""
    s = ""
    i += 1
    while i:
        i, r = divmod(i - 1, 26)
        s = chr(65 + r) + s
    return s


def _num(value) -> Optional[str]:
    """Excel number text, or None if not finite."""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return repr(value) if math.isfinite(value) else None
    return None


_SPACE = ' xml:space="preserve"'


class _SharedStrings:
    def __init__(self) -> None:
        self.index: Dict[str, int] = {}
        self.count = 0

    def add(self, text: str) -> int:
        self.count += 1
        if text not in self.index:
            self.index[text] = len(self.index)
        return self.index[text]

    def xml(self) -> str:
        # xml:space only for text with edge spaces
        items = "".join(f'<si><t{_SPACE if t != t.strip() else ""}>{escape(t)}</t></si>' for t in self.index)
        return (f'{DECL}<sst xmlns="{NS_MAIN}" count="{self.count}" uniqueCount="{len(self.index)}">'
                f'{items}</sst>')


def _cell(ref: str, value, bold: bool, sst: _SharedStrings) -> str:
    style = ' s="1"' if bold else ""
    if value is None or value == "":
        return ""
    if isinstance(value, (bool, int, float)):
        n = _num(value)
        return f'<c r="{ref}"{style}><v>{n}</v></c>' if n is not None else ""
    return f'<c r="{ref}"{style} t="s"><v>{sst.add(str(value))}</v></c>'


def _sheet(rows: Sequence[Sequence], bold_rows: set, sst: _SharedStrings,
           widths: Sequence[float] = (), drawing: bool = False, selected: bool = False) -> str:
    n_cols = max((len(r) for r in rows), default=1) or 1
    dim = f"A1:{_col(n_cols - 1)}{max(len(rows), 1)}"
    tab = ' tabSelected="1"' if selected else ""
    out = [f'{DECL}<worksheet xmlns="{NS_MAIN}" xmlns:r="{NS_R}">',
           f'<dimension ref="{dim}"/>',
           f'<sheetViews><sheetView workbookViewId="0"{tab}/></sheetViews>',
           '<sheetFormatPr defaultRowHeight="15"/>']
    if widths:
        out.append("<cols>" + "".join(
            f'<col min="{i + 1}" max="{i + 1}" width="{w}" customWidth="1"/>' for i, w in enumerate(widths)) + "</cols>")
    out.append("<sheetData>")
    for r, row in enumerate(rows, start=1):
        cells = "".join(_cell(f"{_col(c)}{r}", v, r in bold_rows, sst) for c, v in enumerate(row))
        out.append(f'<row r="{r}">{cells}</row>' if cells else f'<row r="{r}"/>')
    out.append("</sheetData>")
    out.append('<pageMargins left="0.7" right="0.7" top="0.75" bottom="0.75" header="0.3" footer="0.3"/>')
    if drawing:
        out.append('<drawing r:id="rId1"/>')
    out.append("</worksheet>")
    return "".join(out)


def _rich(text: str, size: int) -> str:
    return (f'<c:tx><c:rich><a:bodyPr/><a:lstStyle/><a:p><a:pPr><a:defRPr sz="{size}" b="1"/></a:pPr>'
            f'<a:r><a:rPr lang="en-GB" sz="{size}" b="1"/><a:t>{escape(text)}</a:t></a:r></a:p></c:rich></c:tx>')


def _num_ref(sheet: str, col: str, r0: int, r1: int, values: Sequence) -> str:
    pts = "".join(f'<c:pt idx="{i}"><c:v>{n}</c:v></c:pt>'
                  for i, n in enumerate(_num(v) for v in values) if n is not None)
    return (f"<c:numRef><c:f>'{escape(sheet)}'!${col}${r0}:${col}${r1}</c:f>"
            f'<c:numCache><c:formatCode>General</c:formatCode><c:ptCount val="{len(values)}"/>{pts}'
            f'</c:numCache></c:numRef>')


def _series(idx: int, name: str, sheet: str, xcol: str, ycol: str, r0: int, r1: int,
            xs: Sequence, ys: Sequence, colour: str, dash: str = "solid") -> str:
    return (f'<c:ser><c:idx val="{idx}"/><c:order val="{idx}"/>'
            f'<c:tx><c:v>{escape(name)}</c:v></c:tx>'
            f'<c:spPr><a:ln w="19050" cap="rnd"><a:solidFill><a:srgbClr val="{colour}"/></a:solidFill>'
            f'<a:prstDash val="{dash}"/><a:round/></a:ln></c:spPr>'
            f'<c:marker><c:symbol val="none"/></c:marker>'
            f'<c:xVal>{_num_ref(sheet, xcol, r0, r1, xs)}</c:xVal>'
            f'<c:yVal>{_num_ref(sheet, ycol, r0, r1, ys)}</c:yVal>'
            f'<c:smooth val="0"/></c:ser>')


def _axis(ax_id: int, cross_id: int, pos: str, title: str) -> str:
    return (f'<c:valAx><c:axId val="{ax_id}"/><c:scaling><c:orientation val="minMax"/></c:scaling>'
            f'<c:delete val="0"/><c:axPos val="{pos}"/><c:majorGridlines/>'
            f'<c:title>{_rich(title, 1000)}<c:overlay val="0"/></c:title>'
            f'<c:numFmt formatCode="General" sourceLinked="1"/>'
            f'<c:majorTickMark val="out"/><c:minorTickMark val="none"/><c:tickLblPos val="nextTo"/>'
            f'<c:crossAx val="{cross_id}"/><c:crosses val="autoZero"/><c:crossBetween val="midCat"/></c:valAx>')


def _chart(title: str, series: List[str], x_title: str, y_title: str) -> str:
    return (f'{DECL}<c:chartSpace xmlns:c="{NS_C}" xmlns:a="{NS_A}" xmlns:r="{NS_R}">'
            f'<c:date1904 val="0"/><c:lang val="en-GB"/><c:roundedCorners val="0"/>'
            f'<c:chart><c:title>{_rich(title, 1400)}<c:overlay val="0"/></c:title>'
            f'<c:autoTitleDeleted val="0"/><c:plotArea><c:layout/>'
            f'<c:scatterChart><c:scatterStyle val="lineMarker"/><c:varyColors val="0"/>'
            + "".join(series) +
            f'<c:axId val="10"/><c:axId val="20"/></c:scatterChart>'
            + _axis(10, 20, "b", x_title) + _axis(20, 10, "l", y_title) +
            f'</c:plotArea><c:legend><c:legendPos val="r"/><c:overlay val="0"/></c:legend>'
            f'<c:plotVisOnly val="1"/><c:dispBlanksAs val="gap"/></c:chart>'
            f'<c:printSettings><c:headerFooter/><c:pageMargins b="0.75" l="0.7" r="0.7" t="0.75" '
            f'header="0.3" footer="0.3"/><c:pageSetup/></c:printSettings></c:chartSpace>')


def _anchor(n: int, col: int, row: int, col_to: int, row_to: int) -> str:
    return (f'<xdr:twoCellAnchor editAs="oneCell">'
            f'<xdr:from><xdr:col>{col}</xdr:col><xdr:colOff>0</xdr:colOff>'
            f'<xdr:row>{row}</xdr:row><xdr:rowOff>0</xdr:rowOff></xdr:from>'
            f'<xdr:to><xdr:col>{col_to}</xdr:col><xdr:colOff>0</xdr:colOff>'
            f'<xdr:row>{row_to}</xdr:row><xdr:rowOff>0</xdr:rowOff></xdr:to>'
            f'<xdr:graphicFrame macro=""><xdr:nvGraphicFramePr><xdr:cNvPr id="{n + 1}" name="Chart {n}"/>'
            f'<xdr:cNvGraphicFramePr/></xdr:nvGraphicFramePr>'
            f'<xdr:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/></xdr:xfrm>'
            f'<a:graphic><a:graphicData uri="{NS_C}">'
            f'<c:chart xmlns:c="{NS_C}" xmlns:r="{NS_R}" r:id="rId{n}"/>'
            f'</a:graphicData></a:graphic></xdr:graphicFrame><xdr:clientData/></xdr:twoCellAnchor>')


def _drawing(anchors: Sequence[tuple]) -> str:
    """anchors: (col, row, col_to, row_to) per chart."""
    return (f'{DECL}<xdr:wsDr xmlns:xdr="{NS_XDR}" xmlns:a="{NS_A}">'
            + "".join(_anchor(n, *a) for n, a in enumerate(anchors, start=1)) + '</xdr:wsDr>')


def pick_step(duration_s: float, base_step_s: float = 0.1, max_points: int = 4000) -> float:
    """Time step, widened 1-2-5 to stay under max_points rows."""
    step = base_step_s
    mults = (2, 2.5, 2)   # 0.1 -> 0.2 -> 0.5 -> 1 -> 2 -> 5 ...
    i = 0
    while duration_s / step > max_points:
        step *= mults[i % 3]
        i += 1
    return float(f"{step:.6g}")


def resample_hold(times: Sequence[float], values: Sequence[float],
                  grid: Sequence[float]) -> List[Optional[float]]:
    """Sample-and-hold onto grid; None outside the data."""
    pts = sorted((t, v) for t, v in zip(times, values)
                 if _num(t) is not None and _num(v) is not None)
    out: List[Optional[float]] = []
    if not pts:
        return [None] * len(grid)
    after = [g for g in grid if g >= pts[-1][0] - 1e-9]
    last_until = (after[0] if after else pts[-1][0]) + 1e-9
    j = -1
    for g in grid:
        while j + 1 < len(pts) and pts[j + 1][0] <= g + 1e-9:
            j += 1
        if j < 0 or g > last_until:
            out.append(None)
        else:
            out.append(pts[j][1])
    return out


def scaled_table(log: Sequence[Sequence], time_col: int, bolt_col: int, value_col: int,
                 target: float, maximum: float, step_s: Optional[float] = None):
    """Scaled load sheet. Returns (header, rows, bolt_names, step_s)."""
    bolts: Dict[object, list] = {}
    for r in log:
        bolts.setdefault(r[bolt_col], []).append(r)
    t_end = max((r[time_col] for r in log if _num(r[time_col]) is not None), default=0) or 0
    step = step_s or pick_step(t_end)
    n = int(math.ceil(t_end / step - 1e-9)) + 1
    grid = [round(k * step, 6) for k in range(n)]
    cols = [resample_hold([r[time_col] for r in rs], [r[value_col] for r in rs], grid)
            for rs in bolts.values()]
    names = [f"Bolt {b}" for b in bolts]
    header = ["time_s"] + [f"{nm} (kN)" for nm in names] + ["target_kN", "max_kN"]
    rows = [[g] + [c[k] for c in cols] + [target, maximum] for k, g in enumerate(grid)]
    return header, rows, names, step


STYLES = (f'{DECL}<styleSheet xmlns="{NS_MAIN}">'
          f'<fonts count="2"><font><sz val="11"/><name val="Calibri"/><family val="2"/></font>'
          f'<font><b/><sz val="11"/><name val="Calibri"/><family val="2"/></font></fonts>'
          f'<fills count="2"><fill><patternFill patternType="none"/></fill><fill><patternFill patternType="gray125"/></fill></fills>'
          f'<borders count="1"><border><left/><right/><top/><bottom/><diagonal/></border></borders>'
          f'<cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellStyleXfs>'
          f'<cellXfs count="2"><xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"/>'
          f'<xf numFmtId="0" fontId="1" fillId="0" borderId="0" xfId="0" applyFont="1"/></cellXfs>'
          f'<cellStyles count="1"><cellStyle name="Normal" xfId="0" builtinId="0"/></cellStyles>'
          f'<dxfs count="0"/><tableStyles count="0" defaultTableStyle="TableStyleMedium2" '
          f'defaultPivotStyle="PivotStyleLight16"/></styleSheet>')


def write_report(path: Path, summary: List[list], log_header: List[str], log: List[list],
                 chart_title: str, target_n: float, max_n: float,
                 time_col: int = 0, bolt_col: int = 1, tool_col: int = 2, load_col: int = 4,
                 scaled_col: Optional[int] = None, scaled_target: Optional[float] = None,
                 scaled_max: Optional[float] = None, scaled_unit: str = "kN",
                 step_s: Optional[float] = None) -> None:
    """Summary, then the log grouped by bolt, with a load chart.
    scaled_col adds a full-size chart on a constant time step."""
    rows: List[list] = [list(r) for r in summary]
    rows.append([])
    rows.append(["Load log: every status line from each tool during the run, grouped by bolt"])
    title_row = len(rows)
    rows.append(list(log_header))
    header_row = len(rows)
    rows.extend(log)
    bold = {title_row, header_row}
    # Bold the bolt table header
    for i, r in enumerate(summary, start=1):
        if r and r[0] == "bolt":
            bold.add(i)

    widths = (14, 16, 10, 16, 14, 12, 10, 24, 12)
    has_chart = bool(log)
    sst = _SharedStrings()

    files = {}
    has_scaled = has_chart and scaled_col is not None
    sheets = ["Run"] + (["Chart lines"] if has_chart else []) + (["Scaled load"] if has_scaled else [])
    files["xl/worksheets/sheet1.xml"] = _sheet(rows, bold, sst, widths, drawing=has_chart, selected=True)

    if has_chart:
        t_end = max((r[time_col] for r in log if _num(r[time_col]) is not None), default=1) or 1
        lines = [["time_s", "target_N", "max_N"], [0, target_n, max_n], [t_end, target_n, max_n]]
        files["xl/worksheets/sheet2.xml"] = _sheet(lines, {1}, sst)
        series = []
        x, y = _col(time_col), _col(load_col)
        first = header_row + 1
        start = 0
        for i in range(1, len(log) + 1):
            if i == len(log) or log[i][bolt_col] != log[start][bolt_col]:
                block = log[start:i]
                name = f"Bolt {log[start][bolt_col]} ({log[start][tool_col]})"
                series.append(_series(len(series), name, "Run", x, y, first + start, first + i - 1,
                                      [r[time_col] for r in block], [r[load_col] for r in block],
                                      BOLT_COLOURS[len(series) % len(BOLT_COLOURS)]))
                start = i
        series.append(_series(len(series), "Target (minimum)", "Chart lines", "A", "B", 2, 3,
                              [0, t_end], [target_n, target_n], "2E7D32", "dash"))
        series.append(_series(len(series), "Maximum", "Chart lines", "A", "C", 2, 3,
                              [0, t_end], [max_n, max_n], "D62728", "dash"))
        files["xl/charts/chart1.xml"] = _chart(chart_title, series, "Time since start (s)",
                                               "Stop value (load-cell N)")
        anchors = [(10, 0, 22, 26)]   # columns K..V, rows 1..26

        if has_scaled:
            s_header, s_rows, names, step = scaled_table(
                log, time_col, bolt_col, scaled_col,
                scaled_target if scaled_target is not None else "",
                scaled_max if scaled_max is not None else "", step_s)
            note = [f"{scaled_unit} values resampled every {step:g} s: each row holds the last value "
                    f"the tool had reported at that time (blank before its first and after its last reading)"]
            files["xl/worksheets/sheet3.xml"] = _sheet([note, s_header] + s_rows, {2}, sst,
                                                       [10] + [14] * (len(s_header) - 1))
            r0, r1 = 3, 2 + len(s_rows)
            xs = [r[0] for r in s_rows]
            s_series = []
            for k, nm in enumerate(names):
                s_series.append(_series(k, nm, "Scaled load", "A", _col(k + 1), r0, r1,
                                        xs, [r[k + 1] for r in s_rows], BOLT_COLOURS[k % len(BOLT_COLOURS)]))
            nb = len(names)
            if scaled_target is not None:
                s_series.append(_series(len(s_series), "Target (minimum)", "Scaled load", "A", _col(nb + 1),
                                        r0, r1, xs, [r[nb + 1] for r in s_rows], "2E7D32", "dash"))
            if scaled_max is not None:
                s_series.append(_series(len(s_series), "Maximum", "Scaled load", "A", _col(nb + 2),
                                        r0, r1, xs, [r[nb + 2] for r in s_rows], "D62728", "dash"))
            files["xl/charts/chart2.xml"] = _chart(f"Full-size load vs time ({step:g} s steps)", s_series,
                                                   "Time since start (s)", f"Scaled load ({scaled_unit}, full size)")
            anchors.append((10, 28, 22, 54))   # columns K..V, rows 29..54

        files["xl/drawings/drawing1.xml"] = _drawing(anchors)
        files["xl/drawings/_rels/drawing1.xml.rels"] = (
            f'{DECL}<Relationships xmlns="{NS_PKG_R}">'
            + "".join(f'<Relationship Id="rId{n}" Type="{REL}/chart" Target="../charts/chart{n}.xml"/>'
                      for n in range(1, len(anchors) + 1)) + '</Relationships>')
        files["xl/worksheets/_rels/sheet1.xml.rels"] = (
            f'{DECL}<Relationships xmlns="{NS_PKG_R}">'
            f'<Relationship Id="rId1" Type="{REL}/drawing" Target="../drawings/drawing1.xml"/></Relationships>')

    n = len(sheets)
    files["xl/sharedStrings.xml"] = sst.xml()
    files["xl/styles.xml"] = STYLES
    files["xl/workbook.xml"] = (
        f'{DECL}<workbook xmlns="{NS_MAIN}" xmlns:r="{NS_R}">'
        f'<bookViews><workbookView xWindow="0" yWindow="0" windowWidth="28800" windowHeight="15000" activeTab="0"/></bookViews>'
        f'<sheets>'
        + "".join(f'<sheet name="{escape(s)}" sheetId="{i}" r:id="rId{i}"/>' for i, s in enumerate(sheets, 1))
        + '</sheets><calcPr calcId="191029"/></workbook>')
    files["xl/_rels/workbook.xml.rels"] = (
        f'{DECL}<Relationships xmlns="{NS_PKG_R}">'
        + "".join(f'<Relationship Id="rId{i}" Type="{REL}/worksheet" Target="worksheets/sheet{i}.xml"/>'
                  for i in range(1, n + 1))
        + f'<Relationship Id="rId{n + 1}" Type="{REL}/styles" Target="styles.xml"/>'
        + f'<Relationship Id="rId{n + 2}" Type="{REL}/sharedStrings" Target="sharedStrings.xml"/>'
        + '</Relationships>')

    now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    files["docProps/core.xml"] = (
        f'{DECL}<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties" '
        f'xmlns:dc="http://purl.org/dc/elements/1.1/" xmlns:dcterms="http://purl.org/dc/terms/" '
        f'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">'
        f'<dc:title>{escape(chart_title)}</dc:title><dc:creator>IS-303 torque tool dashboard</dc:creator>'
        f'<dcterms:created xsi:type="dcterms:W3CDTF">{now}</dcterms:created>'
        f'<dcterms:modified xsi:type="dcterms:W3CDTF">{now}</dcterms:modified></cp:coreProperties>')
    files["docProps/app.xml"] = (
        f'{DECL}<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties">'
        f'<Application>IS-303 torque tool dashboard</Application></Properties>')
    files["_rels/.rels"] = (
        f'{DECL}<Relationships xmlns="{NS_PKG_R}">'
        f'<Relationship Id="rId1" Type="{REL}/officeDocument" Target="xl/workbook.xml"/>'
        f'<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" '
        f'Target="docProps/core.xml"/>'
        f'<Relationship Id="rId3" Type="{REL}/extended-properties" Target="docProps/app.xml"/></Relationships>')

    ct = "application/vnd.openxmlformats-officedocument"
    overrides = [("/xl/workbook.xml", f"{ct}.spreadsheetml.sheet.main+xml"),
                 ("/xl/styles.xml", f"{ct}.spreadsheetml.styles+xml"),
                 ("/xl/sharedStrings.xml", f"{ct}.spreadsheetml.sharedStrings+xml"),
                 ("/docProps/core.xml", "application/vnd.openxmlformats-package.core-properties+xml"),
                 ("/docProps/app.xml", f"{ct}.extended-properties+xml")]
    overrides += [(f"/xl/worksheets/sheet{i}.xml", f"{ct}.spreadsheetml.worksheet+xml")
                  for i in range(1, n + 1)]
    if has_chart:
        overrides += [("/xl/drawings/drawing1.xml", f"{ct}.drawing+xml"),
                      ("/xl/charts/chart1.xml", f"{ct}.drawingml.chart+xml")]
    if has_scaled:
        overrides.append(("/xl/charts/chart2.xml", f"{ct}.drawingml.chart+xml"))
    content_types = (
        f'{DECL}<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
        '<Default Extension="xml" ContentType="application/xml"/>'
        + "".join(f'<Override PartName="{p}" ContentType="{c}"/>' for p, c in overrides) + '</Types>')

    # Write to .tmp first, then rename
    tmp = Path(str(path) + ".tmp")
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", content_types)
        for name, data in files.items():
            z.writestr(name, data.encode("utf-8"))
    tmp.replace(path)

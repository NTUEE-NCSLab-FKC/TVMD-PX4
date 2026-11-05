function L = pfa_ulogread(file, topicsWanted)
%PFA_ULOGREAD  Minimal pure-MATLAB ULog (.ulg) reader, no toolbox needed.
%   L = pfa_ulogread(file)                 reads every logged topic
%   L = pfa_ulogread(file, {'vehicle_attitude', ...})   reads only these
%
%   L.t0      start time in µs (first vehicle_status sample, else file header)
%   L.topic   struct: one table per topic instance, named <topic> for
%             instance 0 and <topic>_<k> for instance k. Each table has
%             t (s, relative to L.t0), timestamp (µs, uint64) and every field;
%             array fields (q[4], control[8], ...) become N×n matrix columns,
%             nested types are flattened (esc[0].esc_rpm -> esc_0_esc_rpm).
%   L.params  struct of parameters (last value in the log)
%   L.pchanges table of parameter changes during the log (t, name, value)
%   L.logs    table of logged messages (t, level, text)
%
%   Written because ulogreader (UAV Toolbox R2023a) fails on these PX4 logs
%   with "Colon operands must be in the range of the data type".

if nargin < 2, topicsWanted = {}; end
fid = fopen(file, 'r');
if fid < 0, error('pfa_ulogread: cannot open %s', file); end
d = fread(fid, inf, '*uint8');
fclose(fid);
d = d(:);
magic = uint8([85 76 111 103 1 18 53]);          % 'ULog' 0x01 0x12 0x35
if numel(d) < 16 || any(d(1:7) ~= magic(:))
    error('pfa_ulogread: %s is not a ULog file', file);
end
tHeader = double(typecast(d(9:16), 'uint64'));

n = numel(d);
formats = containers.Map('KeyType', 'char', 'ValueType', 'any');
subName = containers.Map('KeyType', 'double', 'ValueType', 'char');
subMulti = containers.Map('KeyType', 'double', 'ValueType', 'double');
dataId = zeros(1e5, 1); dataOff = zeros(1e5, 1); nd = 0;
params = struct(); pchg = {}; logs = {};
seenData = false; lastTs = 0;

p = 17;
while p + 2 <= n
    sz  = double(d(p)) + 256 * double(d(p + 1));
    typ = char(d(p + 2));
    b = p + 3; e = b + sz - 1;
    if e > n, break; end
    switch typ
        case 'F'
            s = char(d(b:e)');
            k = find(s == ':', 1);
            flds = strsplit(s(k + 1:end), ';');
            formats(s(1:k - 1)) = flds(~cellfun(@isempty, flds));
        case 'A'
            id = double(d(b + 1)) + 256 * double(d(b + 2));
            subName(id) = char(d(b + 3:e)');
            subMulti(id) = double(d(b));
        case 'D'
            nd = nd + 1;
            if nd > numel(dataId)
                dataId(2 * end) = 0; dataOff(2 * end) = 0;   % grow
            end
            dataId(nd) = double(d(b)) + 256 * double(d(b + 1));
            dataOff(nd) = b + 2;                              % first data byte
            seenData = true;
            if sz >= 10
                ts = double(typecast(d(b + 2:b + 9), 'uint64'));
                if lastTs == 0 || (ts > lastTs && ts - lastTs < 1e8), lastTs = ts; end
            end
        case 'P'
            kl = double(d(b));
            key = char(d(b + 1:b + kl)');
            sp = find(key == ' ', 1);
            ty = key(1:sp - 1); nm = key(sp + 1:end);
            vb = d(b + 1 + kl:e);
            switch ty
                case 'float',   v = double(typecast(vb(1:4), 'single'));
                case 'int32_t', v = double(typecast(vb(1:4), 'int32'));
                otherwise,      v = NaN;
            end
            if isvarname(nm), params.(nm) = v; end
            if seenData, pchg(end + 1, :) = {lastTs, nm, v}; end %#ok<AGROW>
        case 'L'
            lvl = double(d(b)) - 48;                          % '0'..'7'
            ts = double(typecast(d(b + 1:b + 8), 'uint64'));
            logs(end + 1, :) = {ts, lvl, char(d(b + 9:e)')}; %#ok<AGROW>
        otherwise
            % I (info), M (multi info), Q (default params), B (flags),
            % S (sync), O (dropout), C, R: not needed here
    end
    p = e + 1;
end
dataId = dataId(1:nd); dataOff = dataOff(1:nd);

% ── decode topics ──
cache = containers.Map('KeyType', 'char', 'ValueType', 'any');   % layouts of this file
L = struct('t0', tHeader, 'topic', struct(), 'params', params);
ids = cell2mat(keys(subName));
% t0: first vehicle_status sample (same origin as QGC / Flight Review plots)
for id = ids
    if strcmp(subName(id), 'vehicle_status') && subMulti(id) == 0
        o = dataOff(find(dataId == id, 1));
        if ~isempty(o), L.t0 = double(typecast(d(o:o + 7), 'uint64')); end
    end
end
for id = ids
    name = subName(id);
    if ~isempty(topicsWanted) && ~any(strcmp(topicsWanted, name)), continue; end
    if ~isKey(formats, name), continue; end
    offs = dataOff(dataId == id);
    if isempty(offs), continue; end
    lay = layout(name, formats, cache);
    T = table();
    for f = 1:numel(lay)
        fl = lay(f);
        if startsWith(fl.name, '_padding'), continue; end
        nb = fl.size * fl.count;
        idx = offs + fl.offset + (0:nb - 1);                 % N×nb byte indices
        B = d(idx);
        if size(B, 2) ~= nb, B = reshape(B, numel(offs), nb); end
        B = B.';                                             % nb×N, little endian per element
        if strcmp(fl.mtype, 'char')
            val = char(B.');
        else
            val = typecast(B(:), fl.mtype);
            val = reshape(val, fl.count, numel(offs)).';
            if ~strcmp(fl.mtype, 'uint64') && ~strcmp(fl.mtype, 'int64'), val = double(val); end
        end
        T.(fl.name) = val;
    end
    if any(strcmp(T.Properties.VariableNames, 'timestamp'))
        T.t = (double(T.timestamp) - L.t0) / 1e6;
        T = movevars(T, 't', 'Before', 1);
    end
    key = name;
    if subMulti(id) > 0, key = sprintf('%s_%d', name, subMulti(id)); end
    L.topic.(matlab.lang.makeValidName(key)) = T;
end
if isempty(pchg)
    L.pchanges = table(zeros(0, 1), cell(0, 1), zeros(0, 1), 'VariableNames', {'t', 'name', 'value'});
else
    L.pchanges = table((cell2mat(pchg(:, 1)) - L.t0) / 1e6, pchg(:, 2), cell2mat(pchg(:, 3)), ...
        'VariableNames', {'t', 'name', 'value'});
end
if isempty(logs)
    L.logs = table(zeros(0, 1), zeros(0, 1), cell(0, 1), 'VariableNames', {'t', 'level', 'text'});
else
    L.logs = table((cell2mat(logs(:, 1)) - L.t0) / 1e6, cell2mat(logs(:, 2)), logs(:, 3), ...
        'VariableNames', {'t', 'level', 'text'});
end
end

% ── helpers ──
function lay = layout(name, formats, cache)
% flat list of fields: name, MATLAB type, byte size, element count, byte offset
% (cache is a handle containers.Map shared within one file)
if isKey(cache, name), lay = cache(name); return; end
base = struct('int8_t', {{'int8', 1}}, 'uint8_t', {{'uint8', 1}}, 'int16_t', {{'int16', 2}}, ...
    'uint16_t', {{'uint16', 2}}, 'int32_t', {{'int32', 4}}, 'uint32_t', {{'uint32', 4}}, ...
    'int64_t', {{'int64', 8}}, 'uint64_t', {{'uint64', 8}}, 'float', {{'single', 4}}, ...
    'double', {{'double', 8}}, 'bool', {{'uint8', 1}}, 'char', {{'char', 1}});
lay = struct('name', {}, 'mtype', {}, 'size', {}, 'count', {}, 'offset', {});
off = 0;
flds = formats(name);
for i = 1:numel(flds)
    parts = strsplit(strtrim(flds{i}), ' ');
    ty = parts{1}; fn = parts{2}; cnt = 1;
    tk = regexp(ty, '^(.*)\[(\d+)\]$', 'tokens', 'once');
    if ~isempty(tk), ty = tk{1}; cnt = str2double(tk{2}); end
    if isfield(base, ty)
        bt = base.(ty);
        lay(end + 1) = struct('name', fn, 'mtype', bt{1}, 'size', bt{2}, 'count', cnt, 'offset', off); %#ok<AGROW>
        off = off + bt{2} * cnt;
    elseif isKey(formats, ty)
        sub = layout(ty, formats, cache);
        subSize = 0;
        for j = 1:numel(sub), subSize = max(subSize, sub(j).offset + sub(j).size * sub(j).count); end
        for c = 1:cnt
            for j = 1:numel(sub)
                if startsWith(sub(j).name, '_padding'), continue; end
                s = sub(j);
                if cnt > 1, s.name = sprintf('%s_%d_%s', fn, c - 1, s.name); else, s.name = sprintf('%s_%s', fn, s.name); end
                s.offset = off + s.offset;
                lay(end + 1) = s; %#ok<AGROW>
            end
            off = off + subSize;
        end
    else
        error('pfa_ulogread: unknown type %s in %s', ty, name);
    end
end
cache(name) = lay;
end

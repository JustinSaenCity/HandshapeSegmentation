import requests
from pathlib import Path
from urllib.parse import urlparse

API = 'https://signcollect.nl/blendBaking/api.php'
CC_BASE = 'https://signcollect.nl/gebarenoverleg_media/fbx/CC/'
OUT = Path('./fbx/palmer/unprocessed')
OUT.mkdir(parents=True, exist_ok=True)

session = requests.Session()


def klaar_sentences():
    page = 1
    while True:
        r = session.get(API, params={
            'action': 'timings',
            'mcpStatusTijdAnnotatie': 'Klaar',
            'page': page,
            'limit': 200,
        }, timeout=120)
        r.raise_for_status()
        d = r.json()
        if not d['success']:
            raise RuntimeError(d['error'])
        yield from d['sentences']
        if page * d['limit'] >= d['total']:
            return
        page += 1


def cc_filename(sentence):
    """Bestandsnaam van de niet-post-processed animatie.
    Aanname: zelfde bestandsnaam als de post-processed fbxUrl, andere map."""
    url = sentence.get('fbxUrl')
    if not url:
        return None
    return Path(urlparse(url).path).name


seen = set()
missing = []
downloaded = 0

for s in klaar_sentences():
    name = cc_filename(s)
    if not name or name in seen:
        continue
    seen.add(name)

    target = OUT / name
    if target.exists():
        continue

    url = CC_BASE + name
    with session.get(url, stream=True, timeout=120) as r:
        if r.status_code == 404:
            missing.append(name)
            print('Niet gevonden:', name)
            continue
        r.raise_for_status()
        tmp = target.with_suffix('.part')
        with open(tmp, 'wb') as f:
            for chunk in r.iter_content(1 << 20):
                f.write(chunk)
        tmp.rename(target)  # pas hernoemen als de download compleet is
    downloaded += 1
    print('Gedownload:', target.name)

print(f'Klaar: {len(seen)} Klaar-animaties, {downloaded} nieuw gedownload, '
      f'{len(missing)} niet gevonden op de server')
if missing:
    (OUT / '_missing.txt').write_text('\n'.join(missing), encoding='utf-8')
    print('Lijst met ontbrekende bestanden: _missing.txt')

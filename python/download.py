import requests
from pathlib import Path

API = 'https://signcollect.nl/blendBaking/api.php'
OUT = Path('./fbx/palmer/processed')
OUT.mkdir(exist_ok=True)

def klaar_sentences():
    page = 1
    while True:
        r = requests.get(API, params={
            'action': 'timings',
            'mcpStatusTijdAnnotatie': 'Klaar',
            'page': page,
            'limit': 200,
        }, timeout=120)
        d = r.json()
        if not d['success']:
            raise RuntimeError(d['error'])
        yield from d['sentences']
        if page * d['limit'] >= d['total']:
            return
        page += 1

seen = set()
for s in klaar_sentences():
    url = s.get('fbxUrl')
    if not url or url in seen:
        continue
    seen.add(url)
    target = OUT / url.rsplit('/', 1)[-1]
    if target.exists():
        continue
    with requests.get(url, stream=True, timeout=120) as r:
        r.raise_for_status()
        with open(target, 'wb') as f:
            for chunk in r.iter_content(1 << 20):
                f.write(chunk)
    print('Gedownload:', target.name)

print(f'Klaar: {len(seen)} FBX-bestanden')
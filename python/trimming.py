import argparse
import math
from pathlib import Path
import sys
 
 
def readMotion(path, boneNames):
    import fbx
    import numpy as np
 
    manager = fbx.FbxManager.Create()
    try:
        settings = fbx.FbxIOSettings.Create(manager, fbx.IOSROOT)
        manager.SetIOSettings(settings)
        importer = fbx.FbxImporter.Create(manager, '')
        scene = fbx.FbxScene.Create(manager, '')
        if not importer.Initialize(str(path), -1, settings) or not importer.Import(scene):
            raise ValueError(f'{path.name}: {importer.GetStatus().GetErrorString()}')
        importer.Destroy()
 
        stackType = fbx.FbxCriteria.ObjectType(fbx.FbxAnimStack.ClassId)
        if scene.GetSrcObjectCount(stackType) != 1:
            raise ValueError(f'{path.name}: expected exactly one FBX animation stack.')
        stack = scene.GetSrcObject(stackType, 0)
        scene.SetCurrentAnimationStack(stack)
 
        nodes = [scene.GetRootNode()]
        bones = []
        while nodes:
            node = nodes.pop()
            if node.GetSkeleton() is not None:
                bones.append(node)
            nodes.extend(node.GetChild(i) for i in range(node.GetChildCount()))
 
        if not boneNames:
            byName = {bone.GetName().lower(): bone.GetName() for bone in bones}
            boneName = next((byName[name] for name in ('head', 'cc_base_hip', 'pelvis', 'hips', 'hip')
                             if name in byName), None)
            if boneName is None:
                raise ValueError(f'{path.name}: could not find a default head or hip bone. '
                                 'Set -bone to an exact bone name. '
                                 f'Available bones include: {", ".join(list(byName.values())[:15])}')
            boneNames = [boneName]
        selected = {}
        for boneName in boneNames:
            matches = [bone for bone in bones if bone.GetName() == boneName]
            if len(matches) != 1:
                raise ValueError(f'{path.name}: expected one bone named {boneName!r}, found {len(matches)}.')
            selected[boneName] = matches[0]
 
        globalSettings = scene.GetGlobalSettings()
        mode = globalSettings.GetTimeMode()
        if mode == fbx.FbxTime.EMode.eCustom:
            fps = fbx.FbxPropertyDouble1(globalSettings.FindProperty('CustomFrameRate')).Get()
        else:
            fps = fbx.FbxTime.GetFrameRate(mode)
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError(f'{path.name}: invalid FBX frame rate.')
 
        # Both endpoints of the declared animation span count as samples.
        span = stack.GetLocalTimeSpan()
        start = span.GetStart().GetSecondDouble()
        duration = span.GetDuration().GetSecondDouble()
        frameCount = math.floor(duration * fps + 0.001) + 1
        units = globalSettings.GetSystemUnit().GetScaleFactor()
        time = fbx.FbxTime()
        rotation = fbx.FbxAMatrix()
        samples = {name: ([], []) for name in selected}
        for frame in range(frameCount):
            time.SetSecondDouble(start + frame / fps)
            for name, bone in selected.items():
                positions, rotations = samples[name]
                transform = bone.EvaluateLocalTransform(time)
                position = transform.GetT()
                rotation.SetQ(transform.GetQ())
                positions.append([position[i] * units for i in range(3)])
                rotations.append([rotation.Get(i, j) for i in range(3) for j in range(3)])
        return fps, {name: (np.asarray(positions), np.asarray(rotations))
                     for name, (positions, rotations) in samples.items()}
    finally:
        manager.Destroy()
 
 
def motionLength(motion):
    return len(next(iter(motion.values()))[0])
 
 
def resampleNearest(motion, sourceFps, targetFps):
    import numpy as np
 
    lastSourceFrame = motionLength(motion) - 1
    # Round the last sample's time onto the target grid, including both endpoints.
    lastTargetFrame = math.floor(lastSourceFrame * targetFps / sourceFps + 0.5)
    sourceFrames = np.floor(np.arange(lastTargetFrame + 1) * sourceFps / targetFps + 0.5)
    sourceFrames = np.minimum(sourceFrames.astype(np.intp), lastSourceFrame)
    return {name: tuple(channel[sourceFrames] for channel in channels)
            for name, channels in motion.items()}
 
 
def findOffset(original, post):
    import numpy as np
 
    originalLength, postLength = motionLength(original), motionLength(post)
    if postLength < 3 or postLength > originalLength:
        raise ValueError('Post-processed motion needs at least three frames and must fit inside the original.')
 
    # Center each window so a constant pose correction cannot shift the match.
    # Normalize translation and rotation separately because their units differ.
    activeBones = {}
    for name, postChannels in post.items():
        channels = []
        for source, target, floor in zip(original[name], postChannels, (1e-5, 1e-4)):
            centered = target - target.mean(axis=0)
            scale = np.sqrt(np.mean(centered ** 2))
            if scale > floor:
                channels.append((source, centered, scale))
        if channels:
            activeBones[name] = channels
    if not activeBones:
        raise ValueError('The selected bones barely move in the post-processed clip. Try -bone with a different bone.')
 
    errors = []
    for offset in range(originalLength - postLength + 1):
        error = 0.0
        for channels in activeBones.values():
            boneError = 0.0
            for source, target, scale in channels:
                window = source[offset:offset + postLength]
                difference = (window - window.mean(axis=0) - target) / scale
                boneError += np.mean(difference ** 2)
            error += boneError / len(channels)
        errors.append(math.sqrt(error / len(activeBones)))
 
    start = int(np.argmin(errors))
    end = originalLength - postLength - start
    tolerance = max(0.01, 0.05 * errors[start])
    nearMatches = sum(error <= errors[start] + tolerance for error in errors)
    return start, end, errors[start], nearMatches, list(activeBones)
 
 
def fitFiles(originalPath, postPath, boneNames, rateMode='auto'):
    originalFps, original = readMotion(originalPath, boneNames)
    postFps, post = readMotion(postPath, list(original))
 
    print(f'Original ({originalPath.name}): {originalFps:g} fps, {motionLength(original)} total frames', flush=True)
    print(f'Post-processed ({postPath.name}): {postFps:g} fps, {motionLength(post)} total frames', flush=True)
    if math.isclose(originalFps, postFps, rel_tol=1e-5):
        fit = findOffset(original, post)
    else:
        modes = ('seconds', 'frames') if rateMode == 'auto' else (rateMode,)
        fits = {}
        failures = {}
        for mode in modes:
            motion = resampleNearest(post, postFps, originalFps) if mode == 'seconds' else post
            try:
                fits[mode] = findOffset(original, motion)
            except ValueError as exc:
                failures[mode] = str(exc)
        if not fits:
            raise ValueError(' | '.join(f'{mode}: {error}' for mode, error in failures.items()))
 
        labels = {'seconds': 'match by elapsed time (nearest-frame resampling)',
                  'frames': 'match frames one-to-one'}
        selected = min(fits, key=lambda mode: fits[mode][2])
        fit = fits[selected]
        print(f'Frame rate handling: {labels[selected]}. Trim counts use original FBX frames.')
        if rateMode == 'auto':
            for mode in modes:
                if mode in fits:
                    start, end, error, _, _ = fits[mode]
                    print(f'  {mode}: error {error:.3f}, trim {start} start / {end} end')
                else:
                    print(f'  {mode}: {failures[mode]}')
            if len(fits) == 2 and fits['seconds'][:2] != fits['frames'][:2]:
                scoreGap = abs(fits['seconds'][2] - fits['frames'][2])
                if scoreGap <= max(0.01, 0.05 * fit[2]):
                    print('Warning: both frame-rate modes score similarly and give different trim counts.')
 
    start, end, error, nearMatches, usedBones = fit
    print(f'{postPath.name} starts {start} frames after {originalPath.name} '
          f'and ends {end} frames before it.')

    # Also write to output/trimming.csv
    trimming_csv = Path('output') / 'trimming.csv'
    trimming_csv.parent.mkdir(exist_ok=True)
    with open(trimming_csv, 'a') as f:
        f.write(f'{originalPath.name},{postPath.name},{start},{end},{error:.3f}\n')
    
    label = 'Bone' if len(usedBones) == 1 else 'Bones'
    print(f'{label}: {", ".join(usedBones)} | normalized match error: {error:.3f}')
    skippedBones = [name for name in post if name not in usedBones]
    if skippedBones:
        print(f'Warning: skipped bones with too little motion: {", ".join(skippedBones)}.')
    if nearMatches > 1:
        print(f'Warning: {nearMatches} offsets have very similar scores. Try another bone to confirm.')
    if error > 0.5:
        print('Warning: weak motion match. Check the source take and try another bone.')
 
 
def main(argv):
    parser = argparse.ArgumentParser(description='Find a trimmed FBX clip within its original take.')
    parser.add_argument('-original', required=True, type=Path)
    parser.add_argument('-post', required=True, type=Path)
    parser.add_argument('-bone', nargs='+', action='extend', metavar='NAME',
                        help='One or more exact bone names. May be repeated. '
                             'Defaults to head, then a recognized hip or pelvis bone.')
    parser.add_argument('--rate-mode', choices=('auto', 'seconds', 'frames'), default='auto',
                        help='When frame rates differ, compare both modes (auto), match elapsed time '
                             '(seconds), or match frames one-to-one (frames).')
    args = parser.parse_args(argv)
    for path in (args.original, args.post):
        if not path.is_file():
            parser.error(f'File not found: {path}')
 
    try:
        boneNames = list(dict.fromkeys(args.bone or []))
        fitFiles(args.original.resolve(), args.post.resolve(), boneNames, args.rate_mode)
    except ImportError as exc:
        parser.exit(1, f'Cannot load the Python dependencies: {exc}. '
                    'Use a Python environment with Autodesk FBX SDK bindings and NumPy installed.\n')
    except (ValueError, RuntimeError) as exc:
        parser.exit(1, f'Error: {exc}\n')
    return 0
 
 
if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))

using OpenKh.Engine;
using OpenKh.Kh2;
using OpenKh.Kh2.Models;
using OpenKh.Tools.Common.Imaging;
using OpenKh.Tools.Kh2ObjectEditor.Services;
using SimpleModelingToolkit.Core.Animations;
using SimpleModelingToolkit.Core.Geometry;
using SimpleModelingToolkit.Core.Materials;
using SimpleModelingToolkit.Core.Nodes;
using SimpleModelingToolkit.Core.Skinning;
using SimpleModelingToolkit.Core.Utils;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;

namespace OpenKh.Tools.Kh2ObjectEditor.Utils
{
    public class SmtConverter
    {
        public static SmtScene GetSmtSceneByFile(string modelFilepath, string textureFilepath, string motionFilepath)
        {
            ModelSkeletal model = ModelSkeletal.Read(File.OpenRead(modelFilepath));
            ModelTexture texture = ModelTexture.Read(File.OpenRead(textureFilepath));
            AnimationBinary  animation = new AnimationBinary(File.OpenRead(motionFilepath));
            return GetSmtScene(model, texture, animation);
        }

        public static SmtScene GetSmtScene()
        {
            return GetSmtScene(null, null, null);
        }
        public static SmtScene GetSmtScene(ModelSkeletal kh2Model, ModelTexture kh2TextureFile, AnimationBinary kh2Animation)
        {
            if(kh2Model == null) kh2Model = MdlxService.Instance.ModelFile;
            if (kh2TextureFile == null) kh2TextureFile = MdlxService.Instance.TextureFile;
            if (kh2Animation == null) kh2Animation = MsetService.Instance.LoadedMotion;

            SmtScene scene = new SmtScene();

            SmtModel model = new SmtModel();
            model.Name = "KH2_MODEL";

            SmtModelNode modelNode = new SmtModelNode(model);
            modelNode.Model = model;

            // Materials
            if (kh2TextureFile != null)
            {
                for (int i = 0; i < kh2TextureFile.Images.Count; i++)
                {
                    SmtMaterial material = new SmtMaterial();
                    material.Name = "Texture" + i;
                    ModelTexture.Texture texture = kh2TextureFile.Images[i];

                    int width = texture.Size.Width;
                    int height = texture.Size.Height;
                    material.DiffuseTextureBitmap = texture.CreateBitmap();

                    scene.MaterialCollection.Add(material);
                }
            }

            // Meshes
            foreach (var kh2Mesh in kh2Model.Groups)
            {
                SmtMesh mesh = new SmtMesh();
                mesh.Name = "Mesh_" + kh2Model.Groups.IndexOf(kh2Mesh);
                mesh.MaterialId = (int)kh2Mesh.Header.TextureIndex;

                foreach(var kh2Vertex in kh2Mesh.Mesh.Vertices)
                {
                    SmtVertex vertex = new SmtVertex();
                    vertex.Position = kh2Vertex.Position;
                    vertex.TextureCoordinates = new Vector2(kh2Vertex.U / 4096.0f, kh2Vertex.V / 4096.0f);
                    vertex.Normals = Vector3.One;

                    foreach(var weightPos in kh2Vertex.BPositions)
                    {
                        SmtJointInfluence weight = new SmtJointInfluence();
                        weight.JointId = weightPos.BoneIndex;
                        weight.Weight = 1;
                        vertex.Weights.Add(weight);
                    }

                    mesh.Vertices.Add(vertex);
                }

                foreach (var kh2Triangle in kh2Mesh.Mesh.Triangles)
                {
                    SmtFace face = new SmtFace();
                    face.VertexIndices = kh2Triangle;
                    mesh.Faces.Add(face);
                }
                scene.MeshCollection.Add(mesh);
                model.Meshes.Add(mesh);
            }

            // Armature
            model.Armature = new SmtArmature();
            foreach(var kh2Joint in kh2Model.Bones)
            {
                SmtJoint joint = new SmtJoint();
                joint.Name = "Joint_" + kh2Model.Bones.IndexOf(kh2Joint);
                joint.ParentId = kh2Joint.ParentIndex;

                Vector3 scale = new Vector3(kh2Joint.ScaleX, kh2Joint.ScaleY, kh2Joint.ScaleZ);
                Vector3 rotation = new Vector3(kh2Joint.RotationX, kh2Joint.RotationY, kh2Joint.RotationZ);
                Vector3 translation = new Vector3(kh2Joint.TranslationX, kh2Joint.TranslationY, kh2Joint.TranslationZ);

                joint.LocalTransform = SmtTools.ComposeXYZ(scale, rotation, translation, true);
                model.Armature.Joints.Add(joint);
            }

            // Animations
            if (kh2Animation != null)
            {
                SmtAnimation animation = new SmtAnimation();

                for (int i = 0; i < kh2Model.BoneCount + kh2Animation.MotionFile.IKHelpers.Count; i++)
                //for (int i = 0; i < kh2Model.BoneCount; i++)
                {
                    animation.JointAnimations.Add(new SmtAnimationJoint(i));
                }

                // IK
                foreach (var ikHelper in kh2Animation.MotionFile.IKHelpers)
                {
                    SmtJoint joint = new SmtJoint();
                    joint.Name = "IK_" + ikHelper.Index;
                    joint.ParentId = ikHelper.ParentId;
                
                    Vector3 scale = new Vector3(ikHelper.ScaleX, ikHelper.ScaleY, ikHelper.ScaleZ);
                    Vector3 rotation = new Vector3(ikHelper.RotateX, ikHelper.RotateY, ikHelper.RotateZ);
                    Vector3 translation = new Vector3(ikHelper.TranslateX, ikHelper.TranslateY, ikHelper.TranslateZ);
                
                    joint.LocalTransform = SmtTools.ComposeXYZ(scale, rotation, translation, true);
                    model.Armature.Joints.Add(joint);
                }

                HashSet<float> keyframeTimes = new HashSet<float>();
                foreach(var time in kh2Animation.MotionFile.KeyTimes) {
                    keyframeTimes.Add(time);
                }
                Dictionary<float, Matrix4x4[]> keyframes = getMatricesForKeyFrames(kh2Model, kh2Animation, keyframeTimes);
                foreach (var time in keyframeTimes)
                {
                    for (int i = 0; i < model.Armature.Joints.Count; i++)
                    {
                        SmtAnimationJoint animJoint = animation.JointAnimations[i];

                        Matrix4x4 jointMatrix = keyframes[time][i];
                        Matrix4x4.Decompose(jointMatrix, out Vector3 scale, out Quaternion rotation, out Vector3 translation);

                        animJoint.AddKeyframeScale(time / 60f, scale);
                        animJoint.AddKeyframeRotation(time / 60f, rotation);
                        animJoint.AddKeyframeTranslation(time / 60f, translation);
                    }
                }

                foreach (var jointAnim in animation.JointAnimations) {
                    jointAnim.SortKeyframes();
                }

                if (!animation.HasNoData()) {
                    model.Armature.Animations.Add(animation);
                }



                
            }

            scene.Models.Add(modelNode);

            return scene;
        }

        // Same as before, but also resolves IK. The previous version modeled IK as bending a 2-bone ancestor
        // chain analytically (law of cosines) with a pole vector guessed from the chain's preferred rotation --
        // that turned out to be the wrong model entirely (verified against Kh2MdlxAssimp.getMatricesForKeyFrames3,
        // itself cross-checked against the reference emulator). Constraints in KH2 do not bend anything at
        // runtime: they simply overwrite a *single* joint's global position/rotation/scale (the actual bend is
        // already baked into the IK Helpers' own animated pose, which comes from Softimage's export, not runtime
        // solving). This version follows that same structure instead:
        //   - Every joint's LOCAL pose (bind + InitialPose + F-Curve) is computed once, up front, for all bones
        //     and IK Helpers alike, storing S/R/T separately so Expressions can read other joints' raw channel
        //     values.
        //   - Then, in the file's own Joints traversal order (parents before children), each joint's Expressions
        //     are evaluated (procedural channel values) followed by its Constraints (which overwrite global
        //     position/rotation/scale).
        //   - Global transforms are resolved recursively through the (mutable) local matrices, so a joint sees
        //     any correction applied earlier in Joints order by its ancestors.
        //   - A joint flagged "calculated" (Joint.Flags & 0x20) with no Constraint of its own is a mid-chain
        //     joint that has to bend to keep the parent-child chain physically connected once one of its
        //     children gets yanked to a Constraint target (e.g. a hip bone bending so a position-constrained
        //     knee/ankle chain reaches its IK Helper target). Bending uses Cyclic Coordinate Descent with a
        //     resolution plane derived from live root/mid/target geometry, per kenjiuno/msetDoc.
        // Only Constraint types Position/Orientation/Scale are applied; Path/Direction/UpVector/TwoPoints/
        // Camera*/Int*/Limiters are not implemented.
        public static Dictionary<float, Matrix4x4[]> getMatricesForKeyFrames(ModelSkeletal model, AnimationBinary animation, HashSet<float> keyframeTimes)
        {
            Motion.InterpolatedMotion motionFile = animation.MotionFile;
            int boneCount = motionFile.InterpolatedMotionHeader.BoneCount;
            int totalJointCount = motionFile.InterpolatedMotionHeader.TotalBoneCount;

            Dictionary<int, List<Motion.FCurve>> curvesByJoint = indexCurvesByJoint(motionFile, boneCount);
            Dictionary<int, List<Motion.InitialPose>> posesByJoint = indexPosesByJoint(motionFile);
            Dictionary<int, List<Motion.Constraint>> constraintsByTarget = indexConstraintsByTarget(motionFile);
            Dictionary<int, List<Motion.Expression>> expressionsByTarget = indexExpressionsByTarget(motionFile);

            // Combined joint space: real bones at [0, boneCount), IK Helpers at [boneCount, totalJointCount).
            // ParentIndex/ParentId of both bones and helpers index directly into this same space.
            int[] parentIds = new int[totalJointCount];
            for (int i = 0; i < boneCount; i++)
                parentIds[i] = model.Bones[i].ParentIndex;
            for (int i = boneCount; i < totalJointCount; i++)
                parentIds[i] = motionFile.IKHelpers[i - boneCount].ParentId;

            // (root, mid, tip) mirrors a classic 2-bone IK chain: root and mid both need to rotate so tip lands
            // exactly on its target. Crucially, tip's own Constraint must NOT also be applied directly to it (see
            // tipHandledByBend below): that constraint exists to supply the bend's target, not to relocate tip
            // independently of its parent - once root/mid are bent correctly, tip reaches the target "for free"
            // through ordinary FK propagation.
            List<(int root, int mid, int tip, int source)> bendChains = new List<(int, int, int, int)>();
            HashSet<int> tipHandledByBend = new HashSet<int>();
            foreach (Motion.Joint joint in motionFile.Joints)
            {
                int mid = joint.JointId;
                if (mid >= boneCount || (joint.Flags & 0x20) == 0 || constraintsByTarget.ContainsKey(mid))
                    continue;

                int root = model.Bones[mid].ParentIndex;
                if (root < 0)
                    continue;

                for (int child = 0; child < boneCount; child++)
                {
                    if (model.Bones[child].ParentIndex != mid || !constraintsByTarget.TryGetValue(child, out List<Motion.Constraint> childConstraints))
                        continue;

                    Motion.Constraint posConstraint = childConstraints.FirstOrDefault(c => (Motion.ConstraintType)c.Type == Motion.ConstraintType.POSITION);
                    if (posConstraint == null)
                        continue;

                    bendChains.Add((root, mid, child, posConstraint.SourceJointId));
                    tipHandledByBend.Add(child);
                    break;
                }
            }

            Dictionary<float, Matrix4x4[]> frameMatrices = new Dictionary<float, Matrix4x4[]>();

            foreach (float keyTime in keyframeTimes)
            {
                Vector3[] scales = new Vector3[totalJointCount];
                Vector3[] rotations = new Vector3[totalJointCount];
                Vector3[] translations = new Vector3[totalJointCount];
                Matrix4x4[] localMatrices = new Matrix4x4[totalJointCount];

                // Pass 1: bind pose + Initial Pose overrides + F-Curve animation, for every joint
                for (int i = 0; i < totalJointCount; i++)
                {
                    (Vector3 Scale, Vector3 Rotation, Vector3 Translation) bindPose = i < boneCount
                        ? getBoneBindPose(model, i)
                        : getHelperBindPose(motionFile.IKHelpers[i - boneCount]);

                    scales[i] = bindPose.Scale;
                    rotations[i] = bindPose.Rotation;
                    translations[i] = bindPose.Translation;

                    if (posesByJoint.TryGetValue(i, out List<Motion.InitialPose> poses))
                        foreach (Motion.InitialPose pose in poses)
                            setChannel(ref scales[i], ref rotations[i], ref translations[i], pose.Channel, pose.Value);

                    if (curvesByJoint.TryGetValue(i, out List<Motion.FCurve> curves))
                        foreach (Motion.FCurve curve in curves)
                            setChannel(ref scales[i], ref rotations[i], ref translations[i], curve.Channel & 0xF, evaluateFCurve(motionFile, curve, keyTime));

                    localMatrices[i] = buildLocalMatrix(scales[i], rotations[i], translations[i]);
                }

                // Pass 2: Expressions then Constraints, per joint, in the file's own Joints traversal order
                foreach (Motion.Joint joint in motionFile.Joints)
                {
                    int jointId = joint.JointId;

                    if (expressionsByTarget.TryGetValue(jointId, out List<Motion.Expression> expressions))
                    {
                        foreach (Motion.Expression expression in expressions)
                        {
                            float value = evaluateExpression(motionFile, scales, rotations, translations, localMatrices, parentIds, keyTime, expression.NodeId);
                            if (expression.TargetChannel is 3 or 4 or 5)
                                value = degToRad(value); // Expressions compute rotation in degrees; channels store radians

                            setChannel(ref scales[jointId], ref rotations[jointId], ref translations[jointId], expression.TargetChannel, value);
                            localMatrices[jointId] = buildLocalMatrix(scales[jointId], rotations[jointId], translations[jointId]);
                        }
                    }

                    if (!tipHandledByBend.Contains(jointId) && constraintsByTarget.TryGetValue(jointId, out List<Motion.Constraint> constraints))
                        foreach (Motion.Constraint constraint in constraints)
                            applyConstraint(motionFile, localMatrices, parentIds, constraint, keyTime);
                }

                // Pass 3: bend the (root, mid) of each chain identified above so 'tip' reaches the position its
                // own (deliberately un-applied, see tipHandledByBend above) Constraint's source joint sits at.
                foreach (var (root, mid, tip, source) in bendChains)
                {
                    Vector3 targetPosition = getGlobalTransform(localMatrices, parentIds, source).Translation;
                    bendTwoBoneChain(localMatrices, parentIds, root, mid, tip, targetPosition);
                }

                frameMatrices.Add(keyTime, localMatrices);
            }

            return frameMatrices;
        }

        private static (Vector3 Scale, Vector3 Rotation, Vector3 Translation) getBoneBindPose(ModelSkeletal model, int boneIndex)
        {
            ModelCommon.Bone bone = model.Bones[boneIndex];
            return (
                new Vector3(bone.ScaleX, bone.ScaleY, bone.ScaleZ),
                new Vector3(bone.RotationX, bone.RotationY, bone.RotationZ),
                new Vector3(bone.TranslationX, bone.TranslationY, bone.TranslationZ));
        }

        private static (Vector3 Scale, Vector3 Rotation, Vector3 Translation) getHelperBindPose(Motion.IKHelper helper) =>
            (
                new Vector3(helper.ScaleX, helper.ScaleY, helper.ScaleZ),
                new Vector3(helper.RotateX, helper.RotateY, helper.RotateZ),
                new Vector3(helper.TranslateX, helper.TranslateY, helper.TranslateZ));

        // Resolves a joint's absolute (world-space) transform by walking up through its (mutable) local matrices.
        // Recursive rather than a single flat pass, since a Constraint can rewrite an ancestor's local matrix
        // *after* a descendant was already visited earlier in Joints order, and later reads must see that update.
        private static Matrix4x4 getGlobalTransform(Matrix4x4[] localMatrices, int[] parentIds, int index)
        {
            if (index < 0)
                return Matrix4x4.Identity;

            int parent = parentIds[index];
            return parent < 0 ? localMatrices[index] : localMatrices[index] * getGlobalTransform(localMatrices, parentIds, parent);
        }

        // Back-solves the local matrix needed so that this joint's *global* transform becomes 'global',
        // given its current parent chain.
        private static void setGlobalTransform(Matrix4x4[] localMatrices, int[] parentIds, int index, Matrix4x4 global)
        {
            int parent = parentIds[index];
            if (parent < 0)
            {
                localMatrices[index] = global;
                return;
            }

            Matrix4x4.Invert(getGlobalTransform(localMatrices, parentIds, parent), out Matrix4x4 parentGlobalInverse);
            localMatrices[index] = global * parentGlobalInverse;
        }

        // FCurvesForward.JointId already matches the combined joint space (bones start at 0); FCurvesInverse.JointId
        // is local to the IK Helpers list and needs the boneCount offset applied to land in the same combined
        // space as everything else.
        private static Dictionary<int, List<Motion.FCurve>> indexCurvesByJoint(Motion.InterpolatedMotion motionFile, int boneCount)
        {
            Dictionary<int, List<Motion.FCurve>> result = new Dictionary<int, List<Motion.FCurve>>();

            void Add(int jointId, Motion.FCurve curve)
            {
                if (!result.TryGetValue(jointId, out List<Motion.FCurve> list))
                    result[jointId] = list = new List<Motion.FCurve>();
                list.Add(curve);
            }

            foreach (Motion.FCurve curve in motionFile.FCurvesForward)
                Add(curve.JointId, curve);
            foreach (Motion.FCurve curve in motionFile.FCurvesInverse)
                Add(curve.JointId + boneCount, curve);

            return result;
        }

        // InitialPose.BoneId is already in the combined joint space (ids >= boneCount are IK Helpers)
        private static Dictionary<int, List<Motion.InitialPose>> indexPosesByJoint(Motion.InterpolatedMotion motionFile)
        {
            Dictionary<int, List<Motion.InitialPose>> result = new Dictionary<int, List<Motion.InitialPose>>();
            foreach (Motion.InitialPose pose in motionFile.InitialPoses)
            {
                if (!result.TryGetValue(pose.BoneId, out List<Motion.InitialPose> list))
                    result[pose.BoneId] = list = new List<Motion.InitialPose>();
                list.Add(pose);
            }
            return result;
        }

        private static Dictionary<int, List<Motion.Constraint>> indexConstraintsByTarget(Motion.InterpolatedMotion motionFile)
        {
            Dictionary<int, List<Motion.Constraint>> result = new Dictionary<int, List<Motion.Constraint>>();
            foreach (Motion.Constraint constraint in motionFile.Constraints)
            {
                if (!result.TryGetValue(constraint.ConstrainedJointId, out List<Motion.Constraint> list))
                    result[constraint.ConstrainedJointId] = list = new List<Motion.Constraint>();
                list.Add(constraint);
            }
            return result;
        }

        private static Dictionary<int, List<Motion.Expression>> indexExpressionsByTarget(Motion.InterpolatedMotion motionFile)
        {
            Dictionary<int, List<Motion.Expression>> result = new Dictionary<int, List<Motion.Expression>>();
            foreach (Motion.Expression expression in motionFile.Expressions)
            {
                if (!result.TryGetValue(expression.TargetId, out List<Motion.Expression> list))
                    result[expression.TargetId] = list = new List<Motion.Expression>();
                list.Add(expression);
            }
            return result;
        }

        private static bool isConstraintActive(Motion.InterpolatedMotion motionFile, Motion.Constraint constraint, float frameTime)
        {
            if (constraint.ActivationCount == 0)
                return true;

            bool active = true;
            for (int i = 0; i < constraint.ActivationCount; i++)
            {
                Motion.ConstraintActivation activation = motionFile.ConstraintActivations[constraint.ActivationStartId + i];
                if (activation.Time > frameTime)
                    break;

                active = activation.Active != 0;
            }
            return active;
        }

        // Applies one Constraint by directly overwriting the target joint's global position/rotation/scale --
        // KH2 constraints do not bend anything at runtime, they just copy a component from the source joint.
        private static void applyConstraint(Motion.InterpolatedMotion motionFile, Matrix4x4[] localMatrices, int[] parentIds, Motion.Constraint constraint, float keyTime)
        {
            if (!isConstraintActive(motionFile, constraint, keyTime))
                return;

            int target = constraint.ConstrainedJointId;
            int source = constraint.SourceJointId;

            switch ((Motion.ConstraintType)constraint.Type)
            {
                case Motion.ConstraintType.POSITION:
                {
                    Matrix4x4 current = getGlobalTransform(localMatrices, parentIds, target);
                    current.Translation = getGlobalTransform(localMatrices, parentIds, source).Translation;
                    setGlobalTransform(localMatrices, parentIds, target, current);
                    break;
                }
                case Motion.ConstraintType.ORIENTATION:
                {
                    Matrix4x4.Decompose(getGlobalTransform(localMatrices, parentIds, target), out Vector3 scale, out _, out Vector3 translation);
                    Matrix4x4.Decompose(getGlobalTransform(localMatrices, parentIds, source), out _, out Quaternion sourceRotation, out _);
                    setGlobalTransform(localMatrices, parentIds, target, Matrix4x4.CreateScale(scale) * Matrix4x4.CreateFromQuaternion(sourceRotation) * Matrix4x4.CreateTranslation(translation));
                    break;
                }
                case Motion.ConstraintType.SCALE:
                {
                    Matrix4x4.Decompose(getGlobalTransform(localMatrices, parentIds, target), out _, out Quaternion rotation, out Vector3 translation);
                    Matrix4x4.Decompose(getGlobalTransform(localMatrices, parentIds, source), out Vector3 sourceScale, out _, out _);
                    setGlobalTransform(localMatrices, parentIds, target, Matrix4x4.CreateScale(sourceScale) * Matrix4x4.CreateFromQuaternion(rotation) * Matrix4x4.CreateTranslation(translation));
                    break;
                }
                default:
                    break; // Path/Direction/UpVector/TwoPoints/Camera*/Int*/Limiters are not implemented
            }
        }

        // Bends 'root' and 'mid' so that 'tip' reaches 'targetPosition', using Cyclic Coordinate Descent (CCD) --
        // per the file format documentation (kenjiuno/msetDoc), this is the actual algorithm KH2 uses for this
        // exact chain shape, not an analytic 2-bone (law of cosines) solve. Each joint is rotated, in turn, by
        // only as much as brings the effector onto the target, repeated over several sweeps until it converges.
        //
        // The rotation axis is recomputed fresh at every single step (standard 3D CCD), not fixed to one plane
        // normal computed once up front: a rotation around a fixed axis preserves every point's coordinate along
        // that axis, so if the chain's un-bent shape isn't already exactly coplanar with (root, mid, target) --
        // generally true, since mid's un-bent orientation has no reason to line up with wherever target happens
        // to be -- a fixed-axis solve leaves a permanent, unresolvable residual along that axis no matter how
        // many sweeps run. Recomputing the axis from the *current* effector/target vectors at each step removes
        // that restriction and lets the solve reach any reachable target in full 3D.
        private static void bendTwoBoneChain(Matrix4x4[] localMatrices, int[] parentIds, int root, int mid, int tip, Vector3 targetPosition)
        {
            Vector3 rootPos = getGlobalTransform(localMatrices, parentIds, root).Translation;
            if (Vector3.Distance(rootPos, targetPosition) < 1e-5f)
                return; // Target sits on the root joint; nothing meaningful to solve

            int[] chain = { root, mid };
            const int sweepCount = 40;

            for (int sweep = 0; sweep < sweepCount; sweep++)
            {
                foreach (int joint in chain)
                {
                    Vector3 pivot = getGlobalTransform(localMatrices, parentIds, joint).Translation;
                    Vector3 toEffector = getGlobalTransform(localMatrices, parentIds, tip).Translation - pivot;
                    Vector3 toTarget = targetPosition - pivot;
                    if (toEffector.LengthSquared() < 1e-10f || toTarget.LengthSquared() < 1e-10f)
                        continue;

                    Vector3 axis = Vector3.Cross(toEffector, toTarget);
                    if (axis.LengthSquared() < 1e-12f)
                        continue; // already aligned (or exactly opposite, which is ambiguous either way)
                    axis = Vector3.Normalize(axis);

                    float angle = MathF.Acos(Math.Clamp(Vector3.Dot(Vector3.Normalize(toEffector), Vector3.Normalize(toTarget)), -1f, 1f));
                    Quaternion delta = Quaternion.CreateFromAxisAngle(axis, angle);
                    setGlobalTransform(localMatrices, parentIds, joint, rotateGlobalAroundOwnPosition(getGlobalTransform(localMatrices, parentIds, joint), delta));
                }
            }
        }

        // Rotates a joint's world-space orientation by 'delta' while keeping its own position fixed (rotating a
        // bone's local rotation alone never moves the bone itself, only its descendants -- see buildLocalMatrix's
        // Scale*Rotation*Translation order -- so this is equivalent to, but easier to compose than, editing Euler angles).
        private static Matrix4x4 rotateGlobalAroundOwnPosition(Matrix4x4 global, Quaternion delta)
        {
            Vector3 pivot = global.Translation;
            return global * Matrix4x4.CreateTranslation(-pivot) * Matrix4x4.CreateFromQuaternion(delta) * Matrix4x4.CreateTranslation(pivot);
        }

        private static void setChannel(ref Vector3 scale, ref Vector3 rotation, ref Vector3 translation, int channel, float value)
        {
            switch (channel)
            {
                case 0:
                    scale.X = value;
                    break;
                case 1:
                    scale.Y = value;
                    break;
                case 2:
                    scale.Z = value;
                    break;
                case 3:
                    rotation.X = value;
                    break;
                case 4:
                    rotation.Y = value;
                    break;
                case 5:
                    rotation.Z = value;
                    break;
                case 6:
                    translation.X = value;
                    break;
                case 7:
                    translation.Y = value;
                    break;
                case 8:
                    translation.Z = value;
                    break;
            }
        }

        // Evaluates a single F-Curve at the given frame time, using the Constant/Linear/Hermite keys Softimage exported.
        // Tangents are used as-is (not scaled by segment length), matching Kh2MotionEngine's existing FCurve evaluator.
        private static float evaluateFCurve(Motion.InterpolatedMotion motionFile, Motion.FCurve curve, float frameTime)
        {
            if (curve.KeyCount == 0)
                return 0f;

            for (int index = curve.KeyCount - 1; index >= 0; index--)
            {
                Motion.Key leftKey = motionFile.FCurveKeys[curve.KeyStartId + index];
                float leftTime = motionFile.KeyTimes[(ushort)leftKey.Type_Time >> 2];

                if (index > 0 && frameTime < leftTime)
                    continue;

                float leftValue = motionFile.KeyValues[leftKey.ValueId];

                // Past the last key, wrap back to the first one rather than holding flat: harmless when queried
                // exactly at (or past) the last key's own time, since n below then evaluates to 0 (or is clamped by
                // the rightTime<=leftTime guard), returning leftValue regardless of what the wrap target is.
                Motion.Key rightKey = index + 1 < curve.KeyCount
                    ? motionFile.FCurveKeys[curve.KeyStartId + index + 1]
                    : motionFile.FCurveKeys[curve.KeyStartId];
                float rightTime = motionFile.KeyTimes[(ushort)rightKey.Type_Time >> 2];
                float rightValue = motionFile.KeyValues[rightKey.ValueId];

                if (rightTime <= leftTime)
                    return leftValue;

                float n = (frameTime - leftTime) / (rightTime - leftTime);

                switch ((Motion.KeyType)(leftKey.Type_Time & 3))
                {
                    case Motion.KeyType.LINEAR:
                        return MathEx.Lerp(leftValue, rightValue, n);
                    case Motion.KeyType.HERMITE:
                        float tangentOut = leftKey.RightTangentId >= 0 ? motionFile.KeyTangents[leftKey.RightTangentId] : 0f;
                        float tangentIn = rightKey.LeftTangentId >= 0 ? motionFile.KeyTangents[rightKey.LeftTangentId] : 0f;
                        return MathEx.CubicHermite(n, leftValue, rightValue, tangentOut, tangentIn);
                    default: // CONSTANT
                        return leftValue;
                }
            }

            return motionFile.KeyValues[motionFile.FCurveKeys[curve.KeyStartId].ValueId];
        }

        private static Matrix4x4 buildLocalMatrix(Vector3 scale, Vector3 rotation, Vector3 translation) =>
            Matrix4x4.CreateScale(scale) *
            Matrix4x4.CreateRotationX(rotation.X) *
            Matrix4x4.CreateRotationY(rotation.Y) *
            Matrix4x4.CreateRotationZ(rotation.Z) *
            Matrix4x4.CreateTranslation(translation);

        private static float radToDeg(float radians) => radians * (180f / MathF.PI);
        private static float degToRad(float degrees) => degrees * (MathF.PI / 180f);

        // ExpressionNode.Data packs Type (byte, bits 0-7), IsGlobal (bit 8) and Element (short, bits 16-31).
        // Motion.ExpressionNode.type only decodes bits 12-15, so it can't be used here -- decode Type/Element directly.
        private static Motion.ExpressionType getExpressionNodeType(Motion.ExpressionNode node) => (Motion.ExpressionType)(node.Data & 0xFF);
        private static int getExpressionNodeElement(Motion.ExpressionNode node) => (short)(node.Data >> 16);

        private static bool isExpressionListNode(Motion.InterpolatedMotion motionFile, int index) =>
            index >= 0 && index < motionFile.ExpressionNodes.Count && getExpressionNodeType(motionFile.ExpressionNodes[index]) == Motion.ExpressionType.LIST;

        // Evaluates a Softimage expression tree (as used to drive IK Helper channels procedurally instead of a
        // plain F-Curve). Trig functions and rotation channel readback/output operate in degrees, matching Softimage;
        // channels themselves store radians, hence the degToRad/radToDeg conversions sprinkled through this switch.
        private static float evaluateExpression(
            Motion.InterpolatedMotion motionFile, Vector3[] scales, Vector3[] rotations, Vector3[] translations,
            Matrix4x4[] localMatrices, int[] parentIds, float keyTime, int index)
        {
            if (index < 0 || index >= motionFile.ExpressionNodes.Count)
                return 0f;

            Motion.ExpressionNode node = motionFile.ExpressionNodes[index];
            Motion.ExpressionType nodeType = getExpressionNodeType(node);

            float Evaluate(int childIndex) => evaluateExpression(motionFile, scales, rotations, translations, localMatrices, parentIds, keyTime, childIndex);

            switch (nodeType)
            {
                case Motion.ExpressionType.FUNC_SIN: return MathF.Sin(degToRad(Evaluate(node.CAR)));
                case Motion.ExpressionType.FUNC_COS: return MathF.Cos(degToRad(Evaluate(node.CAR)));
                case Motion.ExpressionType.FUNC_TAN: return MathF.Tan(degToRad(Evaluate(node.CAR)));
                case Motion.ExpressionType.FUNC_ASIN:
                {
                    float val = Evaluate(node.CAR);
                    if (val >= 1f) return 90f;
                    if (val <= -1f) return -90f;
                    return radToDeg(MathF.Asin(val));
                }
                case Motion.ExpressionType.FUNC_ACOS:
                {
                    float val = Evaluate(node.CAR);
                    if (val >= 1f) return 0f;
                    if (val <= -1f) return 180f;
                    return radToDeg(MathF.Acos(val));
                }
                case Motion.ExpressionType.FUNC_ATAN: return radToDeg(MathF.Atan(Evaluate(node.CAR)));
                case Motion.ExpressionType.FUNC_LOG: return MathF.Log(Evaluate(node.CAR));
                case Motion.ExpressionType.FUNC_EXP: return MathF.Exp(Evaluate(node.CAR));
                case Motion.ExpressionType.FUNC_ABS: return MathF.Abs(Evaluate(node.CAR));
                case Motion.ExpressionType.FUNC_POW: return MathF.Pow(Evaluate(node.CAR), Evaluate(node.CDR));
                case Motion.ExpressionType.FUNC_SQRT: return MathF.Sqrt(MathF.Abs(Evaluate(node.CAR)));
                case Motion.ExpressionType.FUNC_MIN: return Math.Min(Evaluate(node.CAR), Evaluate(node.CDR));
                case Motion.ExpressionType.FUNC_MAX: return Math.Max(Evaluate(node.CAR), Evaluate(node.CDR));
                case Motion.ExpressionType.FUNC_AV:
                {
                    if (node.CAR >= 0 && isExpressionListNode(motionFile, node.CAR) && node.CDR < 0)
                    {
                        List<float> list = evaluateExpressionList(motionFile, scales, rotations, translations, localMatrices, parentIds, keyTime, node.CAR);
                        return list.Count == 0 ? 0f : list.Sum() / list.Count;
                    }
                    return 0f;
                }
                case Motion.ExpressionType.FUNC_COND:
                {
                    if (node.CAR >= 0 && isExpressionListNode(motionFile, node.CAR) && node.CDR < 0)
                    {
                        List<float> list = evaluateExpressionList(motionFile, scales, rotations, translations, localMatrices, parentIds, keyTime, node.CAR);
                        if (list.Count == 3)
                            return list[0] >= 0 ? list[1] : list[2];
                    }
                    return 0f;
                }
                case Motion.ExpressionType.FUNC_AT_FRAME:
                case Motion.ExpressionType.FUNC_AT_FRAME_ROT:
                {
                    float time = Evaluate(node.CAR) * 60f / motionFile.InterpolatedMotionHeader.FrameData.FramesPerSecond;
                    int curveIndex = (int)Evaluate(node.CDR);
                    Motion.FCurve curve = curveIndex < motionFile.FCurvesForward.Count
                        ? motionFile.FCurvesForward[curveIndex]
                        : motionFile.FCurvesInverse[curveIndex - motionFile.FCurvesForward.Count];
                    float value = evaluateFCurve(motionFile, curve, time);
                    return nodeType == Motion.ExpressionType.FUNC_AT_FRAME_ROT ? radToDeg(value) : value;
                }
                case Motion.ExpressionType.FUNC_CTR_DIST:
                {
                    int totalCount = scales.Length;
                    int bone1 = node.CAR < 0 || node.CAR >= totalCount ? 0 : node.CAR;
                    int bone2 = node.CDR < 0 || node.CDR >= totalCount ? 0 : node.CDR;
                    Vector3 pos1 = getGlobalTransform(localMatrices, parentIds, bone1).Translation;
                    Vector3 pos2 = getGlobalTransform(localMatrices, parentIds, bone2).Translation;
                    return Vector3.Distance(pos1, pos2);
                }
                case Motion.ExpressionType.FUNC_FMOD: return Evaluate(node.CAR) % Evaluate(node.CDR);
                case Motion.ExpressionType.OP_PLUS: return Evaluate(node.CAR) + Evaluate(node.CDR);
                case Motion.ExpressionType.OP_MINUS: return Evaluate(node.CAR) - Evaluate(node.CDR);
                case Motion.ExpressionType.OP_MUL: return Evaluate(node.CAR) * Evaluate(node.CDR);
                case Motion.ExpressionType.OP_DIV: return Evaluate(node.CAR) / Evaluate(node.CDR);
                case Motion.ExpressionType.OP_MOD: return MathF.Round(Evaluate(node.CAR)) % MathF.Round(Evaluate(node.CDR));
                case Motion.ExpressionType.OP_EQ: return Evaluate(node.CAR) == Evaluate(node.CDR) ? 1f : 0f;
                case Motion.ExpressionType.OP_GT: return Evaluate(node.CAR) > Evaluate(node.CDR) ? 1f : 0f;
                case Motion.ExpressionType.OP_GE: return Evaluate(node.CAR) >= Evaluate(node.CDR) ? 1f : 0f;
                case Motion.ExpressionType.OP_LT: return Evaluate(node.CAR) < Evaluate(node.CDR) ? 1f : 0f;
                case Motion.ExpressionType.OP_LE: return Evaluate(node.CAR) <= Evaluate(node.CDR) ? 1f : 0f;
                case Motion.ExpressionType.OP_AND: return Evaluate(node.CAR) >= 1f && Evaluate(node.CDR) >= 1f ? 1f : 0f;
                case Motion.ExpressionType.OP_OR: return Evaluate(node.CAR) >= 1f || Evaluate(node.CDR) >= 1f ? 1f : 0f;
                case Motion.ExpressionType.VARIABLE_FC: return keyTime;
                case Motion.ExpressionType.CONSTANT_NUM: return node.Value;
                case Motion.ExpressionType.FCURVE_ETRNX: return getExpressionChannel(translations, node).X;
                case Motion.ExpressionType.FCURVE_ETRNY: return getExpressionChannel(translations, node).Y;
                case Motion.ExpressionType.FCURVE_ETRNZ: return getExpressionChannel(translations, node).Z;
                case Motion.ExpressionType.FCURVE_ROTX: return radToDeg(getExpressionChannel(rotations, node).X);
                case Motion.ExpressionType.FCURVE_ROTY: return radToDeg(getExpressionChannel(rotations, node).Y);
                case Motion.ExpressionType.FCURVE_ROTZ: return radToDeg(getExpressionChannel(rotations, node).Z);
                case Motion.ExpressionType.FCURVE_SCALX: return getExpressionChannel(scales, node).X;
                case Motion.ExpressionType.FCURVE_SCALY: return getExpressionChannel(scales, node).Y;
                case Motion.ExpressionType.FCURVE_SCALZ: return getExpressionChannel(scales, node).Z;
                case Motion.ExpressionType.ELEMENT_NAME: return 0f; // Documented as a no-op
                default: return 0f; // LIST is only ever consumed by FUNC_AV/FUNC_COND, never evaluated directly
            }
        }

        private static Vector3 getExpressionChannel(Vector3[] channel, Motion.ExpressionNode node)
        {
            int element = getExpressionNodeElement(node);
            return element >= 0 && element < channel.Length ? channel[element] : Vector3.Zero;
        }

        private static List<float> evaluateExpressionList(
            Motion.InterpolatedMotion motionFile, Vector3[] scales, Vector3[] rotations, Vector3[] translations,
            Matrix4x4[] localMatrices, int[] parentIds, float keyTime, int index)
        {
            Motion.ExpressionNode node = motionFile.ExpressionNodes[index];
            List<float> list = new List<float>();

            void AddBranch(int childIndex)
            {
                if (childIndex < 0)
                    return;

                if (isExpressionListNode(motionFile, childIndex))
                    list.AddRange(evaluateExpressionList(motionFile, scales, rotations, translations, localMatrices, parentIds, keyTime, childIndex));
                else
                    list.Add(evaluateExpression(motionFile, scales, rotations, translations, localMatrices, parentIds, keyTime, childIndex));
            }

            AddBranch(node.CAR);
            AddBranch(node.CDR);
            return list;
        }
    }
}

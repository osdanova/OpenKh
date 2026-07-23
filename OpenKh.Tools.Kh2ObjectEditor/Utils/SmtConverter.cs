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

                // Constraints
                //foreach(var constraint in kh2Animation.MotionFile.Constraints)
                //{
                //    if(constraint.Type == 0)
                //    {
                //        SmtAnimationJoint sourceJoint = animation.JointAnimations[constraint.SourceJointId];
                //        SmtAnimationJoint constrainedJoint = animation.JointAnimations[constraint.ConstrainedJointId];
                //
                //        foreach(float time in keyframeTimes)
                //        {
                //            Vector3 pos = sourceJoint.Keyframes[time].Translation.Value;
                //            int parentId = model.Armature.Joints[constraint.ConstrainedJointId].ParentId;
                //            SmtAnimationJoint parentJoint = animation.JointAnimations[parentId];
                //            pos = pos - parentJoint.Keyframes[time].Translation.Value;
                //
                //            constrainedJoint.Keyframes[time].Translation = pos;
                //        }
                //        constrainedJoint.Keyframes = sourceJoint.Keyframes;
                //    }
                //}

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

        public static Dictionary<float, Matrix4x4[]> getMatricesForKeyFrames(ModelSkeletal model, AnimationBinary animation, HashSet<float> keyframeTimes)
        {
            Motion.InterpolatedMotion motionFile = animation.MotionFile;
            Dictionary<float, Matrix4x4[]> frameMatrices = new Dictionary<float, Matrix4x4[]>();

            foreach (float keyTime in keyframeTimes)
                frameMatrices.Add(keyTime, computeBonePose(model, motionFile, keyTime));

            return frameMatrices;
        }

        // Computes the absolute (world-space) FK matrices of the real skeleton bones at the given frame time.
        // Assumes bones are ordered so a parent always precedes its children.
        private static Matrix4x4[] computeBonePose(ModelSkeletal model, Motion.InterpolatedMotion motionFile, float keyTime)
        {
            int baseBoneCount = motionFile.InterpolatedMotionHeader.BoneCount;
            int totalJointCount = motionFile.InterpolatedMotionHeader.BoneCount + motionFile.IKHelpers.Count;

            Vector3[] scales = new Vector3[totalJointCount];
            Vector3[] rotations = new Vector3[totalJointCount];
            Vector3[] translations = new Vector3[totalJointCount];
            for (int i = 0; i < scales.Count(); i++) {
                scales[i] = Vector3.One;
            }

            // Base pose, taken from the model's bind pose
            for (int i = 0; i < baseBoneCount; i++)
            {
                ModelCommon.Bone bone = model.Bones[i];
                scales[i] = new Vector3(bone.ScaleX, bone.ScaleY, bone.ScaleZ);
                rotations[i] = new Vector3(bone.RotationX, bone.RotationY, bone.RotationZ);
                translations[i] = new Vector3(bone.TranslationX, bone.TranslationY, bone.TranslationZ);
            }

            // IK helper base pose, taken from each helper's own rest transform
            for (int i = 0; i < motionFile.IKHelpers.Count; i++)
            {
                Motion.IKHelper ikHelper = motionFile.IKHelpers[i];
                int jointId = baseBoneCount + i;
                scales[jointId] = new Vector3(ikHelper.ScaleX, ikHelper.ScaleY, ikHelper.ScaleZ);
                rotations[jointId] = new Vector3(ikHelper.RotateX, ikHelper.RotateY, ikHelper.RotateZ);
                translations[jointId] = new Vector3(ikHelper.TranslateX, ikHelper.TranslateY, ikHelper.TranslateZ);
            }

            // Initial pose overrides
            foreach (Motion.InitialPose pose in motionFile.InitialPoses)
            {
                if (pose.BoneId >= baseBoneCount)
                    continue; // IK Helper, not a real bone

                setChannel(scales, rotations, translations, pose.BoneId, pose.Channel, pose.Value);
            }

            // Per-frame F-Curve overrides
            foreach (Motion.FCurve curve in motionFile.FCurvesForward)
            {
                if (curve.JointId >= baseBoneCount)
                    continue; // IK Helper, not a real bone

                float value = evaluateFCurve(motionFile, curve, keyTime);
                setChannel(scales, rotations, translations, curve.JointId, curve.Channel & 0xF, value);
            }
            foreach (Motion.FCurve curve in motionFile.FCurvesInverse)
            {
                int jointId = motionFile.InterpolatedMotionHeader.BoneCount + curve.JointId;
            
                float value = evaluateFCurve(motionFile, curve, keyTime);
                setChannel(scales, rotations, translations, jointId, curve.Channel & 0xF, value);
            }

            Matrix4x4[] matrices = new Matrix4x4[totalJointCount];
            for (int i = 0; i < totalJointCount; i++)
            {
                Matrix4x4 localMatrix = buildLocalMatrix(scales[i], rotations[i], translations[i]);
                matrices[i] = localMatrix;
            }

            return matrices;
        }

        private static void setChannel(Vector3[] scales, Vector3[] rotations, Vector3[] translations, int jointId, int channel, float value)
        {
            Vector3 scale = scales[jointId];
            Vector3 rotation = rotations[jointId];
            Vector3 translation = translations[jointId];

            setChannel(ref scale, ref rotation, ref translation, channel, value);

            scales[jointId] = scale;
            rotations[jointId] = rotation;
            translations[jointId] = translation;
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
    }
}

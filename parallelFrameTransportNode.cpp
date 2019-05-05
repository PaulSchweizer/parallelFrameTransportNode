#include <algorithm>
#include <string.h>
#include <maya/MIOStream.h>
#include <math.h>

#include <maya/MPxNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnPlugin.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MTypeId.h>
#include <maya/MPlug.h>
#include <maya/MVector.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MQuaternion.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MMatrix.h>
#include <maya/MRampAttribute.h>

class parallelFrameTransport : public MPxNode
{
	public:
		parallelFrameTransport();
		~parallelFrameTransport() override;

		MStatus compute(const MPlug& plug, MDataBlock& data) override;

		static void* creator();
		static MStatus  initialize();

		static MObject inputCurve;
		static MObject restLength;
		static MObject params;
		static MObject startMatrix;
		static MObject twist;
		static MObject absoluteParam;
		static MObject scaleRamp;

		static MObject outTranslate;
		static MObject outRotateX;
		static MObject outRotateY;
		static MObject outRotateZ;
		static MObject outRotate;
		static MObject outScale;
		static MTypeId id;

	private:
		static double degToRad;
};

MTypeId parallelFrameTransport::id(0x00130a00);
MObject parallelFrameTransport::inputCurve;
MObject parallelFrameTransport::restLength;
MObject parallelFrameTransport::params;
MObject parallelFrameTransport::startMatrix;
MObject parallelFrameTransport::twist;
MObject parallelFrameTransport::absoluteParam;
MObject parallelFrameTransport::scaleRamp;

MObject parallelFrameTransport::outTranslate;
MObject parallelFrameTransport::outRotateX;
MObject parallelFrameTransport::outRotateY;
MObject parallelFrameTransport::outRotateZ;
MObject parallelFrameTransport::outRotate;
MObject parallelFrameTransport::outScale;

double parallelFrameTransport::degToRad = M_PI / 180;

parallelFrameTransport::parallelFrameTransport() {}
parallelFrameTransport::~parallelFrameTransport() {}

void* parallelFrameTransport::creator()
{
	return new parallelFrameTransport();
}

MStatus parallelFrameTransport::compute(const MPlug& plug, MDataBlock& data)
{
	MStatus status;

	if (plug == outTranslate || plug == outRotate || plug == outScale ||
		plug.parent() == outTranslate || plug.parent() == outRotate || plug.parent() == outScale)
	{
		// Input handles
		MArrayDataHandle inputDataParams = data.inputValue(params, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MDataHandle inputDataCurve = data.inputValue(inputCurve, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MFnNurbsCurve curveFn(inputDataCurve.asNurbsCurveTransformed(), &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MDataHandle inputDataStartMatrix = data.inputValue(startMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MRampAttribute inputDataTwist(thisMObject(), twist, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MDataHandle inputDataAbsoluteParam = data.inputValue(absoluteParam, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MRampAttribute inputDataScaleRamp(thisMObject(), scaleRamp, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MDataHandle inputRestLength = data.inputValue(restLength, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Output handles
		MArrayDataHandle outTranslateHandle = data.outputValue(outTranslate, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MArrayDataHandle outRotateHandle = data.outputValue(outRotate, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MArrayDataHandle outScaleHandle = data.outputValue(outScale, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Vars
		double param;
		MPoint position;
		MDataHandle handle;
		MVector tangent;
		MQuaternion q;
		MVector normal;
		MVector binormal;
		double theta;
		MMatrix m;
		double rot[3];
		MTransformationMatrix::RotationOrder rotOrder;
		MMatrix StartMatrix = inputDataStartMatrix.asMatrix();
		double curveLength = curveFn.length();
		bool absParam = inputDataAbsoluteParam.asBool();
		float twistRampValue;
		float scaleRampValue;
		double curveRestLength = inputRestLength.asDouble();
		double lengthRatio = pow(curveRestLength / curveLength, 0.5) - 1;

		MVector prevNormal = MVector(StartMatrix[0][0], StartMatrix[0][1], StartMatrix[0][2]);
		MVector prevTangent = MVector(StartMatrix[1][0], StartMatrix[1][1], StartMatrix[1][2]);

		for (unsigned int i = 0; i < inputDataParams.elementCount(); i++)
		{
			param = inputDataParams.outputValue().asDouble();

			// Param depending on length if absoluteParam is enabled
			if (absParam) {
				param = curveFn.findParamFromLength(curveLength * param);
			}

			// Translate
			curveFn.getPointAtParam(param, position);
			handle = outTranslateHandle.outputValue();
			handle.set(position.x, position.y, position.z);

			// Parallel Frame Transportation
			tangent = curveFn.tangent(param).normal();
			binormal = tangent ^ prevTangent;
			if (binormal.length() == 0)
			{
				normal = prevNormal;
			}
			else
			{
				binormal.normalize();
				theta = acos(prevTangent * tangent);
				q.setAxisAngle(binormal, theta);
				normal = q * prevNormal;
			}
			binormal = (normal ^ tangent).normal();
			prevNormal = normal;
			prevTangent = tangent;

			// Construct TransformMatrix to extract rotation
			m[0][0] = normal.x;
			m[0][1] = normal.y;
			m[0][2] = normal.z;
			m[1][0] = tangent.x;
			m[1][1] = tangent.y;
			m[1][2] = tangent.z;
			m[2][0] = binormal.x;
			m[2][1] = binormal.y;
			m[2][2] = binormal.z;
			MTransformationMatrix transform = MTransformationMatrix(m);

			// Apply twist
			inputDataTwist.getValueAtPosition(param, twistRampValue, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);
			q.setAxisAngle(MVector().yAxis, twistRampValue * degToRad);
			transform.rotateBy(q, MSpace::kObject);

			// Set rotation
			transform.getRotation(rot, rotOrder);
			handle = outRotateHandle.outputValue();
			handle.set(rot[0], rot[1], rot[2]);

			// Scale for volume preservation
			inputDataScaleRamp.getValueAtPosition(param, scaleRampValue, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);
			handle = outScaleHandle.outputValue();
			handle.set(std::max(0.0, 1 + lengthRatio * scaleRampValue));

			// Nexts
			if (i + 1 < inputDataParams.elementCount())
			{
				CHECK_MSTATUS_AND_RETURN_IT(inputDataParams.next());
				CHECK_MSTATUS_AND_RETURN_IT(outTranslateHandle.next());
				CHECK_MSTATUS_AND_RETURN_IT(outRotateHandle.next());
				CHECK_MSTATUS_AND_RETURN_IT(outScaleHandle.next());
			}
		}
		data.setClean(plug);
		return MS::kSuccess;
	}
	else
	{
		return MS::kUnknownParameter;
	}
}


MStatus parallelFrameTransport::initialize()
{
	MStatus status;
	MFnNumericAttribute nAttr;
	MFnUnitAttribute uAttr;
	MFnTypedAttribute tAttr;
	MFnMatrixAttribute mAttr;

	inputCurve = tAttr.create("inputCurve", "c", MFnData::kNurbsCurve);
	nAttr.setChannelBox(true);

	restLength = nAttr.create("restLength", "rl", MFnNumericData::kDouble, 0.0);
	nAttr.setChannelBox(true);

	params = nAttr.create("params", "p", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true);
	nAttr.setChannelBox(true);

	startMatrix = mAttr.create("startMatrix", "sm", MFnMatrixAttribute::kDouble);

	twist = MRampAttribute::createCurveRamp("twist", "t");
	nAttr.setChannelBox(true);

	scaleRamp = MRampAttribute::createCurveRamp("scaleRamp", "sr");
	nAttr.setChannelBox(true);

	absoluteParam = nAttr.create("absoluteParam", "a", MFnNumericData::kBoolean, true);
	nAttr.setChannelBox(true);

	outTranslate = nAttr.create("outTranslate", "ot", MFnNumericData::k3Double, 0.0);
	nAttr.setArray(true);
	nAttr.setWritable(true);

	outRotateX = uAttr.create("outRotateX", "rx", MFnUnitAttribute::kAngle, 0.0);
	outRotateY = uAttr.create("outRotateY", "ry", MFnUnitAttribute::kAngle, 0.0);
	outRotateZ = uAttr.create("outRotateZ", "rz", MFnUnitAttribute::kAngle, 0.0);
	outRotate = nAttr.create("outRotate", "r", outRotateX, outRotateY, outRotateZ);
	nAttr.setArray(true);
	nAttr.setWritable(true);

	outScale = nAttr.create("outScale", "os", MFnNumericData::kDouble, 0.0);
	nAttr.setArray(true);
	nAttr.setWritable(true);

	// Inputs
	status = addAttribute(inputCurve);
	if (!status) { status.perror("addAttribute(inputCurve)"); return status; }
	status = addAttribute(params);
	if (!status) { status.perror("addAttribute(params)"); return status; }
	status = addAttribute(restLength);
	if (!status) { status.perror("addAttribute(restLength)"); return status; }
	status = addAttribute(startMatrix);
	if (!status) { status.perror("addAttribute(startMatrix)"); return status; }
	status = addAttribute(twist);
	if (!status) { status.perror("addAttribute(twist)"); return status; }
	status = addAttribute(scaleRamp);
	if (!status) { status.perror("addAttribute(scaleRamp)"); return status; }
	status = addAttribute(absoluteParam);
	if (!status) { status.perror("addAttribute(absoluteParam)"); return status; }

	// Outputs
	status = addAttribute(outTranslate);
	if (!status) { status.perror("addAttribute(outTranslate)"); return status; }
	status = addAttribute(outRotate);
	if (!status) { status.perror("addAttribute(outRotate)"); return status; }
	status = addAttribute(outScale);
	if (!status) { status.perror("addAttribute(outScale)"); return status; }

	// Inputs affecting outputs
	status = attributeAffects(inputCurve, outTranslate);
	if (!status) { status.perror("attributeAffects(inputCurve, outTranslate)"); return status; }
	status = attributeAffects(params, outTranslate);
	if (!status) { status.perror("attributeAffects(params, outTranslate)"); return status; }
	status = attributeAffects(restLength, outTranslate);
	if (!status) { status.perror("attributeAffects(restLength, outTranslate)"); return status; }
	status = attributeAffects(startMatrix, outTranslate);
	if (!status) { status.perror("attributeAffects(startMatrix, outTranslate)"); return status; }
	status = attributeAffects(absoluteParam, outTranslate);
	if (!status) { status.perror("attributeAffects(absoluteParam, outTranslate)"); return status; }

	status = attributeAffects(inputCurve, outRotate);
	if (!status) { status.perror("attributeAffects(inputCurve, outRotate)"); return status; }
	status = attributeAffects(params, outRotate);
	if (!status) { status.perror("attributeAffects(params, outRotate)"); return status; }
	status = attributeAffects(restLength, outRotate);
	if (!status) { status.perror("attributeAffects(restLength, outRotate)"); return status; }
	status = attributeAffects(startMatrix, outRotate);
	if (!status) { status.perror("attributeAffects(startMatrix, outRotate)"); return status; }
	status = attributeAffects(twist, outRotate);
	if (!status) { status.perror("attributeAffects(twist, outRotate)"); return status; }
	status = attributeAffects(absoluteParam, outRotate);
	if (!status) { status.perror("attributeAffects(absoluteParam, outRotate)"); return status; }

	status = attributeAffects(inputCurve, outScale);
	if (!status) { status.perror("attributeAffects(inputCurve, outScale)"); return status; }
	status = attributeAffects(params, outScale);
	if (!status) { status.perror("attributeAffects(params, outScale)"); return status; }
	status = attributeAffects(restLength, outScale);
	if (!status) { status.perror("attributeAffects(restLength, outScale)"); return status; }
	status = attributeAffects(startMatrix, outScale);
	if (!status) { status.perror("attributeAffects(startMatrix, outScale)"); return status; }
	status = attributeAffects(absoluteParam, outScale);
	if (!status) { status.perror("attributeAffects(absoluteParam, outScale)"); return status; }
	status = attributeAffects(scaleRamp, outScale);
	if (!status) { status.perror("attributeAffects(scaleRamp, outScale)"); return status; }

	return MS::kSuccess;
}

MStatus initializePlugin(MObject obj)
{
	MStatus status;
	MFnPlugin plugin(obj, "https://github.com/PaulSchweizer", "1.0", "Any");

	status = plugin.registerNode(
		"parallelFrameTransport",
		parallelFrameTransport::id,
		&parallelFrameTransport::creator,
		&parallelFrameTransport::initialize);
	if (!status) {
		status.perror("registerNode");
	}
	return status;
}

MStatus uninitializePlugin(MObject obj)
{
	MStatus status;
	MFnPlugin plugin(obj);

	status = plugin.deregisterNode(parallelFrameTransport::id);
	if (!status) {
		status.perror("deregisterNode");
	}
	return status;
}

def BMValues(stereo):
    """returns dictionnary with the stereo parameters"""
    dic = { "numDisparities":stereo.getNumDisparities(),
            "blockSize":stereo.getBlockSize(),
            "preFilterType":stereo.getPreFilterType(),
            "preFilterSize":stereo.getPreFilterSize(),
            "preFilterCap":stereo.getPreFilterCap(),
            "textureThreshold":stereo.getTextureThreshold(),
            "uniquenessRatio":stereo.getUniquenessRatio(),
            "speckleRange":stereo.getSpeckleRange(),
            "speckleWindowSize":stereo.getSpeckleWindowSize(),
            "disp12MaxDiff":stereo.getDisp12MaxDiff(),
            "minDisparity":stereo.getMinDisparity()}
    return dic

# def BMSetValues(stereo, dic):
#     stereo.setNumDisparities(numDisparities)
#     stereo.setBlockSize(blockSize)
#     stereo.setPreFilterType(preFilterType)
#     stereo.setPreFilterSize(preFilterSize)
#     stereo.setPreFilterCap(preFilterCap)
#     stereo.setTextureThreshold(textureThreshold)
#     stereo.setUniquenessRatio(uniquenessRatio)
#     stereo.setSpeckleRange(speckleRange)
#     stereo.setSpeckleWindowSize(speckleWindowSize)
#     stereo.setDisp12MaxDiff(disp12MaxDiff)
#     stereo.setMinDisparity(minDisparity)

def BMValuesToText(stereo):
    numDisparities=stereo.getNumDisparities()
    blockSize=stereo.getBlockSize()
    preFilterType=stereo.getPreFilterType()
    preFilterSize=stereo.getPreFilterSize()
    preFilterCap=stereo.getPreFilterCap()
    textureThreshold=stereo.getTextureThreshold()
    uniquenessRatio=stereo.getUniquenessRatio()
    speckleRange=stereo.getSpeckleRange()
    speckleWindowSize=stereo.getSpeckleWindowSize()
    disp12MaxDiff=stereo.getDisp12MaxDiff()
    minDisparity=stereo.getMinDisparity()
    text = "num:"+str(numDisparities)+", block:"+str(blockSize)+", filter:"+str(preFilterType)+", f_Sz:"+str(preFilterSize)+", f_Cap:"+str(preFilterCap)+", texture:"+str(textureThreshold)+", uRatio"+str(uniquenessRatio)+", sp_Range:"+str(speckleRange)+", sp_Sz:"+str(speckleWindowSize)+", disp:"+str(disp12MaxDiff)+", min:"+str(minDisparity)
    return text
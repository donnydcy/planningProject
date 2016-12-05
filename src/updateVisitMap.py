def updateVisitMap(visitMap,uavPath,ugvPath,deployFlag):
	if not deployFlag:
		visitMap[uavPath[:,1],uavPath[:,0]] += 1
		visitMap[ugvPath[:,1],ugvPath[:,0]] += 1
	else
		visitMap[ugvPath[:,1],ugvPath[:,0]] += 1
	return visitMap
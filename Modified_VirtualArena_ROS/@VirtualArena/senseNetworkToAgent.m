function readings = senseNetworkToAgent(obj,timeInfo,systemId)
            
            nSensor = length(obj.sensorsNetwork)/2;
            emptyMeasurements = 1;
            
            readings = cell(nSensor,1);
            for i =1:nSensor
                
                sensor  = obj.sensorsNetwork{(i-1)*2+1};
                LsensorH = obj.sensorsNetwork{(i-1)*2+2};
                Lsensor = LsensorH(timeInfo);
                detectableAgents  = Lsensor(systemId,:);
                indexesDetectable = 1:length(Lsensor(systemId,:));
                indexesDetectable = indexesDetectable(logical(detectableAgents));
                
                readings{i} = sensor.sense(timeInfo,systemId,obj.systemsList{systemId},obj.systemsList(indexesDetectable),indexesDetectable);
                
                if not(isempty(readings{i}))
                    emptyMeasurements = 0;
                end
                
            end
            
            if emptyMeasurements
                readings = {};
            else
                readings = {readings};
            end
            
        end
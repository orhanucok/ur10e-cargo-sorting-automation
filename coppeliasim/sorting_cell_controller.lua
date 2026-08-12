sim = require 'sim'
simIK = require 'simIK'

local function smoothstep(value)
    local u = math.max(0.0, math.min(1.0, value))
    return u * u * (3.0 - 2.0 * u)
end

local function mix(a, b, u)
    return {
        a[1] + (b[1] - a[1]) * u,
        a[2] + (b[2] - a[2]) * u,
        a[3] + (b[3] - a[3]) * u,
    }
end

local function distance(a, b)
    local dx = a[1] - b[1]
    local dy = a[2] - b[2]
    local dz = a[3] - b[3]
    return math.sqrt(dx * dx + dy * dy + dz * dz)
end

local function references(tag)
    return sim.getReferencedHandles(selfHandle, tag)
end

local function setTarget(robot, position)
    sim.setObjectPosition(robot.target, sim.handle_world, position)
end

local function targetReached(robot, position, tolerance)
    return distance(sim.getObjectPosition(robot.tip, sim.handle_world), position) <= tolerance
end

local function configureIk(robot)
    robot.ikEnvironment = simIK.createEnvironment()
    robot.ikGroup = simIK.createGroup(robot.ikEnvironment)
    simIK.setGroupCalculation(
        robot.ikEnvironment,
        robot.ikGroup,
        simIK.method_damped_least_squares,
        0.065,
        40
    )
    local _, handleMap = simIK.addElementFromScene(
        robot.ikEnvironment,
        robot.ikGroup,
        robot.base,
        robot.tip,
        robot.target,
        simIK.constraint_pose
    )
    robot.ikHandleMap = handleMap
end

local function driveIk(robot, index)
    simIK.syncFromSim(robot.ikEnvironment, {robot.ikGroup})
    local result, reason = simIK.handleGroup(
        robot.ikEnvironment,
        robot.ikGroup,
        {allowError = true}
    )
    sim.setInt32Signal('sorting_cell_ik_result_' .. index, result)
    sim.setInt32Signal('sorting_cell_ik_reason_' .. index, reason)
    for _, joint in ipairs(robot.joints) do
        local desired = simIK.getJointPosition(robot.ikEnvironment, robot.ikHandleMap[joint])
        sim.setJointTargetPosition(joint, desired)
    end
end

local function updateBelt(dt)
    local travel = beltSpeed * dt
    for index, slat in ipairs(beltSlats) do
        local phase = (beltPhases[index] + travel) % beltPathLength
        beltPhases[index] = phase
        local x, z
        if phase < beltLength then
            x = beltMinimumX + phase
            z = beltTopZ
        elseif phase < beltLength + beltDrop then
            x = beltMinimumX + beltLength
            z = beltTopZ - (phase - beltLength)
        elseif phase < 2.0 * beltLength + beltDrop then
            x = beltMinimumX + beltLength - (phase - beltLength - beltDrop)
            z = beltTopZ - beltDrop
        else
            x = beltMinimumX
            z = beltTopZ - beltDrop + (phase - 2.0 * beltLength - beltDrop)
        end
        local p = sim.getObjectPosition(slat, sim.handle_world)
        sim.setObjectPosition(slat, sim.handle_world, {x, p[2], z})
    end
end

local function updateCameraDecisions()
    for box, route in pairs(boxRoutes) do
        if not classified[box] and not heldBy[box] then
            local p = sim.getObjectPosition(box, sim.handle_world)
            if p[1] >= cameraDecisionX then
                classified[box] = true
                local destination = route == 1 and 'KIRMIZI' or 'MAVI'
                sim.addLog(
                    sim.verbosity_scriptinfos,
                    string.format('[Basler-sim] parcel=%d color-label=%s assignment=%s-arm', box, destination, destination)
                )
            end
        end
    end
end

local function collisionRobotIndex()
    for index, robot in ipairs(robots) do
        local collision = sim.checkCollision(robot.collisionCollection, conveyorCollection)
        if collision > 0 then return index end
    end
    return 0
end

local function findCandidate(robot)
    local best = nil
    local bestDistance = 1000.0
    for box, route in pairs(boxRoutes) do
        if route == robot.route and classified[box] and not heldBy[box] and not completed[box] then
            local p = sim.getObjectPosition(box, sim.handle_world)
            local dx = math.abs(p[1] - robot.pick[1])
            if dx < robot.pickTolerance and math.abs(p[2]) < 0.34 and p[3] > 0.70 then
                if dx < bestDistance then
                    best = box
                    bestDistance = dx
                end
            end
        end
    end
    return best
end

local function attachBox(robot)
    if not robot.box then return false end
    local tipPosition = sim.getObjectPosition(robot.tip, sim.handle_world)
    local boxPosition = sim.getObjectPosition(robot.box, sim.handle_world)
    local contact = {boxPosition[1], boxPosition[2], boxPosition[3] + boxHalfHeight}
    if distance(tipPosition, contact) > 0.32 then
        return false
    end
    sim.setObjectInt32Param(robot.box, sim.shapeintparam_static, 1)
    sim.setObjectParent(robot.box, robot.tip, true)
    heldBy[robot.box] = robot.name
    if supplyIndex and supplyIndex[robot.box] then
        supplyPhase[robot.box] = 'sorter'
    end
    sim.addLog(sim.verbosity_scriptinfos, '[' .. robot.name .. '] vakum kavrama OK')
    return true
end

local function releaseBox(robot)
    if not robot.box then return end
    local releasedBox = robot.box
    sim.setObjectParent(releasedBox, cellRoot, true)
    sim.setObjectInt32Param(releasedBox, sim.shapeintparam_static, 0)
    sim.resetDynamicObject(releasedBox)
    heldBy[releasedBox] = nil
    completed[releasedBox] = true
    if supplyIndex and supplyIndex[releasedBox] then
        supplyPhase[releasedBox] = 'sorted'
        supplySortedAt[releasedBox] = sim.getSimulationTime()
        supplyCompletedCount = supplyCompletedCount + 1
        sim.setInt32Signal('sorting_cell_supply_completed', supplyCompletedCount)
    end
    local destination = robot.route == 1 and 'KIRMIZI' or 'MAVI'
    sim.addLog(sim.verbosity_scriptinfos, '[' .. robot.name .. '] parcel placed in the ' .. destination .. ' bin')
    robot.box = nil
end

local function updateSorter(robot, now)
    local elapsed = now - robot.stateStarted

    if robot.state == 'wait' then
        setTarget(robot, robot.hoverPick)
        robot.box = findCandidate(robot)
        if robot.box then
            robot.state = 'descend'
            robot.stateStarted = now
        end
    elseif robot.state == 'descend' then
        local boxPosition = sim.getObjectPosition(robot.box, sim.handle_world)
        local contact = {boxPosition[1], boxPosition[2], boxPosition[3] + boxHalfHeight + 0.008}
        setTarget(robot, mix(robot.hoverPick, contact, smoothstep(elapsed / 1.0)))
        if elapsed >= 1.0 and targetReached(robot, contact, 0.10) then
            if attachBox(robot) then
                robot.pickPosition = contact
                robot.state = 'lift'
                robot.stateStarted = now
            else
                robot.box = nil
                robot.state = 'wait'
                robot.stateStarted = now
            end
        end
    elseif robot.state == 'lift' then
        setTarget(robot, mix(robot.pickPosition, robot.hoverPick, smoothstep(elapsed / 1.1)))
        if elapsed >= 1.1 and targetReached(robot, robot.hoverPick, 0.12) then
            robot.state = 'transfer'
            robot.stateStarted = now
        end
    elseif robot.state == 'transfer' then
        setTarget(robot, mix(robot.hoverPick, robot.hoverDrop, smoothstep(elapsed / 2.0)))
        if elapsed >= 2.0 and targetReached(robot, robot.hoverDrop, 0.14) then
            robot.state = 'lower'
            robot.stateStarted = now
        end
    elseif robot.state == 'lower' then
        setTarget(robot, mix(robot.hoverDrop, robot.drop, smoothstep(elapsed / 0.9)))
        if elapsed >= 0.9 and targetReached(robot, robot.drop, 0.10) then
            releaseBox(robot)
            robot.state = 'raise_after_drop'
            robot.stateStarted = now
        end
    elseif robot.state == 'raise_after_drop' then
        setTarget(robot, mix(robot.drop, robot.hoverDrop, smoothstep(elapsed / 1.0)))
        if elapsed >= 1.0 and targetReached(robot, robot.hoverDrop, 0.12) then
            robot.state = 'return'
            robot.stateStarted = now
        end
    elseif robot.state == 'return' then
        setTarget(robot, mix(robot.hoverDrop, robot.hoverPick, smoothstep(elapsed / 2.0)))
        if elapsed >= 2.0 and targetReached(robot, robot.hoverPick, 0.14) then
            robot.state = 'wait'
            robot.stateStarted = now
        end
    elseif robot.state == 'recover_home' then
        setTarget(robot, mix(robot.recoveryStart, robot.hoverPick, smoothstep(elapsed / 1.8)))
        if elapsed >= 1.8 and targetReached(robot, robot.hoverPick, 0.15) then
            robot.state = robot.box and 'transfer' or 'wait'
            robot.stateStarted = now
        end
    end

    if
        robot.state ~= 'wait' and robot.state ~= 'recover_home' and
        now - robot.stateStarted >= motionWatchdogTimeout
    then
        sorterWatchdogCount = sorterWatchdogCount + 1
        sim.setInt32Signal('sorting_cell_sorter_watchdog_recoveries', sorterWatchdogCount)
        sim.addLog(sim.verbosity_scriptwarnings, '[' .. robot.name .. '] hareket zaman asimi; guvenli yukseklige donuyor')
        if robot.box and not heldBy[robot.box] then robot.box = nil end
        robot.recoveryStart = sim.getObjectPosition(robot.tip, sim.handle_world)
        robot.state = 'recover_home'
        robot.stateStarted = now
    end
end

local function setParcelVisibility(box, visibilityLayer)
    for _, shape in ipairs(sim.getObjectsInTree(box, sim.object_shape_type, 0)) do
        sim.setObjectInt32Param(shape, sim.objintparam_visibility_layer, visibilityLayer)
    end
end

local function boxVerticalExtent(box)
    local matrix = sim.getObjectMatrix(box, sim.handle_world)
    return
        math.abs(matrix[9]) * 0.155 +
        math.abs(matrix[10]) * 0.115 +
        math.abs(matrix[11]) * boxHalfHeight
end

local function boxTopContact(box)
    local p = sim.getObjectPosition(box, sim.handle_world)
    return {p[1], p[2], p[3] + boxVerticalExtent(box) + 0.008}
end

local function labelNormalZ(box)
    return sim.getObjectMatrix(box, sim.handle_world)[11]
end

local function transitionInfeed(robot, state, now)
    robot.state = state
    robot.stateStarted = now
    sim.setStringSignal('sorting_cell_infeed_state', state)
end

local function stageSupplyBox(box)
    local index = supplyIndex[box]
    local stagingPosition = sim.getObjectPosition(supplyStaging, sim.handle_world)
    sim.setObjectParent(box, cellRoot, true)
    sim.setObjectInt32Param(box, sim.shapeintparam_static, 1)
    setParcelVisibility(box, 0)
    sim.setObjectPosition(
        box,
        sim.handle_world,
        {stagingPosition[1], stagingPosition[2], stagingPosition[3] - (index - 1) * 0.25}
    )
    sim.setObjectOrientation(box, sim.handle_world, {0.0, 0.0, 0.0})
    supplyActive[box] = nil
    supplyLandedAt[box] = nil
    supplyBeltAt[box] = nil
    supplySortedAt[box] = nil
    supplyPhase[box] = 'staged'
    boxRoutes[box] = nil
    classified[box] = nil
    completed[box] = nil
    heldBy[box] = nil
end

local function releaseSupplyBox(box, now)
    local index = supplyIndex[box]
    local spawnPosition = sim.getObjectPosition(supplySpawn, sim.handle_world)
    local chuteOrientation = sim.getObjectOrientation(supplyChute, sim.handle_world)
    local xJitter = ((index - 1) % 3 - 1) * 0.035
    local yawJitter = math.rad(((index * 37) % 50) - 25)

    sim.setObjectParent(box, cellRoot, true)
    sim.setObjectInt32Param(box, sim.shapeintparam_static, 1)
    sim.setObjectPosition(
        box,
        sim.handle_world,
        {spawnPosition[1] + xJitter, spawnPosition[2], spawnPosition[3]}
    )
    sim.setObjectOrientation(box, sim.handle_world, {chuteOrientation[1], 0.0, yawJitter})
    setParcelVisibility(box, 1)
    sim.setObjectInt32Param(box, sim.shapeintparam_static, 0)
    sim.resetDynamicObject(box)
    supplyActive[box] = now
    supplyLandedAt[box] = nil
    supplyPhase[box] = 'feeding'
    sim.addLog(
        sim.verbosity_scriptinfos,
        string.format('[INFEED] magazine released parcel %02d', index)
    )
end

local function findInfeedCandidate(robot, now)
    local best = nil
    local bestScore = -1000.0
    local basePosition = sim.getObjectPosition(robot.base, sim.handle_world)
    for _, box in ipairs(supplyBoxes) do
        if
            supplyPhase[box] == 'bin' and supplyLandedAt[box] and
            now - supplyLandedAt[box] >= 0.80 and not heldBy[box]
        then
            local p = sim.getObjectPosition(box, sim.handle_world)
            local dx = p[1] - basePosition[1]
            local dy = p[2] - basePosition[2]
            local horizontalReach = math.sqrt(dx * dx + dy * dy)
            local reachable = p[2] <= 0.32 and horizontalReach <= 1.22
            local score = p[3] - horizontalReach * 0.08
            if reachable and score > bestScore then
                best = box
                bestScore = score
            end
        end
    end
    return best
end

local function attachInfeedBox(robot)
    if not robot.box then return false end
    local contact = boxTopContact(robot.box)
    local tipPosition = sim.getObjectPosition(robot.tip, sim.handle_world)
    if distance(tipPosition, contact) > 0.36 then return false end
    sim.setObjectInt32Param(robot.box, sim.shapeintparam_static, 1)
    sim.setObjectParent(robot.box, robot.tip, true)
    heldBy[robot.box] = robot.name
    supplyPhase[robot.box] = 'infeed'
    supplyLandedAt[robot.box] = nil
    sim.setInt32Signal('sorting_cell_infeed_vacuum', 1)
    sim.addLog(sim.verbosity_scriptinfos, '[UR10e-INFEED] vacuum pick from bin OK')
    return true
end

local function placeInfeedBoxOnStation(robot)
    sim.setObjectParent(robot.box, cellRoot, true)
    sim.setObjectInt32Param(robot.box, sim.shapeintparam_static, 1)
    heldBy[robot.box] = nil
    supplyPhase[robot.box] = 'station'
    sim.setInt32Signal('sorting_cell_infeed_vacuum', 0)
    sim.addLog(sim.verbosity_scriptinfos, '[UR10e-INFEED] parcel placed at inspection station')
end

local function releaseInfeedBoxToBelt(robot, now)
    local box = robot.box
    sim.setObjectParent(box, cellRoot, true)
    sim.setObjectInt32Param(box, sim.shapeintparam_static, 0)
    sim.resetDynamicObject(box)
    heldBy[box] = nil
    boxRoutes[box] = supplyRoute[box]
    classified[box] = nil
    completed[box] = nil
    supplyPhase[box] = 'belt'
    supplyBeltAt[box] = now
    infeedCompletedCount = infeedCompletedCount + 1
    sim.setInt32Signal('sorting_cell_infeed_completed', infeedCompletedCount)
    sim.setInt32Signal('sorting_cell_infeed_vacuum', 0)
    sim.addLog(
        sim.verbosity_scriptinfos,
        string.format('[UR10e-INFEED] oriented parcel %02d transferred to conveyor', supplyIndex[box])
    )
    robot.box = nil
end

local function updateInfeed(robot, now)
    local elapsed = now - robot.stateStarted

    if robot.state == 'wait_bin' then
        setTarget(robot, robot.hoverBin)
        if targetReached(robot, robot.hoverBin, 0.14) then
            robot.box = findInfeedCandidate(robot, now)
            if robot.box then
                robot.pickContact = boxTopContact(robot.box)
                transitionInfeed(robot, 'descend_bin', now)
            end
        end
    elseif robot.state == 'descend_bin' then
        robot.pickContact = boxTopContact(robot.box)
        setTarget(robot, mix(robot.hoverBin, robot.pickContact, smoothstep(elapsed / 1.0)))
        if elapsed >= 1.0 and targetReached(robot, robot.pickContact, 0.10) then
            if attachInfeedBox(robot) then
                robot.pickContact = sim.getObjectPosition(robot.tip, sim.handle_world)
                transitionInfeed(robot, 'lift_bin', now)
            else
                robot.box = nil
                transitionInfeed(robot, 'wait_bin', now)
            end
        end
    elseif robot.state == 'lift_bin' then
        setTarget(robot, mix(robot.pickContact, robot.hoverBin, smoothstep(elapsed / 0.9)))
        if elapsed >= 0.9 and targetReached(robot, robot.hoverBin, 0.12) then
            transitionInfeed(robot, 'transfer_station', now)
        end
    elseif robot.state == 'transfer_station' then
        setTarget(robot, mix(robot.hoverBin, robot.hoverStation, smoothstep(elapsed / 1.3)))
        if elapsed >= 1.3 and targetReached(robot, robot.hoverStation, 0.12) then
            local boxPosition = sim.getObjectPosition(robot.box, sim.handle_world)
            local tipPosition = sim.getObjectPosition(robot.tip, sim.handle_world)
            local desiredCenter = {
                robot.stationCenter[1],
                robot.stationCenter[2],
                robot.stationSurface + boxVerticalExtent(robot.box) + 0.005,
            }
            robot.stationPlaceTarget = {
                tipPosition[1] + desiredCenter[1] - boxPosition[1],
                tipPosition[2] + desiredCenter[2] - boxPosition[2],
                tipPosition[3] + desiredCenter[3] - boxPosition[3],
            }
            transitionInfeed(robot, 'lower_station', now)
        end
    elseif robot.state == 'lower_station' then
        setTarget(robot, mix(robot.hoverStation, robot.stationPlaceTarget, smoothstep(elapsed / 0.8)))
        if elapsed >= 0.8 and targetReached(robot, robot.stationPlaceTarget, 0.09) then
            placeInfeedBoxOnStation(robot)
            transitionInfeed(robot, 'raise_inspection', now)
        end
    elseif robot.state == 'raise_inspection' then
        setTarget(robot, mix(robot.stationPlaceTarget, robot.hoverStation, smoothstep(elapsed / 0.75)))
        if elapsed >= 0.75 and targetReached(robot, robot.hoverStation, 0.12) then
            transitionInfeed(robot, 'inspect_label', now)
        end
    elseif robot.state == 'inspect_label' then
        setTarget(robot, robot.hoverStation)
        if elapsed >= 0.55 then
            robot.labelWasUp = labelNormalZ(robot.box) >= 0.92
            local result = robot.labelWasUp and 'USTTE' or 'YANLIS-YONDE'
            sim.addLog(sim.verbosity_scriptinfos, '[D455-INFEED] label=' .. result)
            robot.stationContact = boxTopContact(robot.box)
            transitionInfeed(robot, 'descend_station', now)
        end
    elseif robot.state == 'descend_station' then
        robot.stationContact = boxTopContact(robot.box)
        setTarget(robot, mix(robot.hoverStation, robot.stationContact, smoothstep(elapsed / 0.85)))
        if elapsed >= 0.85 and targetReached(robot, robot.stationContact, 0.10) then
            if attachInfeedBox(robot) then
                robot.stationContact = sim.getObjectPosition(robot.tip, sim.handle_world)
                transitionInfeed(robot, 'lift_station', now)
            else
                transitionInfeed(robot, 'inspect_label', now)
            end
        end
    elseif robot.state == 'lift_station' then
        setTarget(robot, mix(robot.stationContact, robot.hoverStation, smoothstep(elapsed / 0.85)))
        if elapsed >= 0.85 and targetReached(robot, robot.hoverStation, 0.12) then
            if robot.labelWasUp then
                transitionInfeed(robot, 'transfer_belt', now)
            else
                robot.rotationStartOrientation = sim.getObjectOrientation(robot.box, sim.handle_world)
                robot.rotationStartPosition = sim.getObjectPosition(robot.box, sim.handle_world)
                transitionInfeed(robot, 'rotate_label_up', now)
            end
        end
    elseif robot.state == 'rotate_label_up' then
        setTarget(robot, robot.hoverStation)
        local u = smoothstep(elapsed / 1.15)
        local startOrientation = robot.rotationStartOrientation
        sim.setObjectOrientation(
            robot.box,
            sim.handle_world,
            {startOrientation[1] * (1.0 - u), startOrientation[2] * (1.0 - u), startOrientation[3]}
        )
        local tipPosition = sim.getObjectPosition(robot.tip, sim.handle_world)
        local vacuumPosition = {
            tipPosition[1],
            tipPosition[2],
            tipPosition[3] - boxHalfHeight - 0.008,
        }
        sim.setObjectPosition(
            robot.box,
            sim.handle_world,
            mix(robot.rotationStartPosition, vacuumPosition, u)
        )
        if elapsed >= 1.15 and targetReached(robot, robot.hoverStation, 0.12) then
            orientationCorrectionCount = orientationCorrectionCount + 1
            sim.setInt32Signal('sorting_cell_orientation_corrections', orientationCorrectionCount)
            transitionInfeed(robot, 'verify_label', now)
        end
    elseif robot.state == 'verify_label' then
        setTarget(robot, robot.hoverStation)
        if elapsed >= 0.35 then
            local verified = labelNormalZ(robot.box) >= 0.98
            sim.addLog(
                verified and sim.verbosity_scriptinfos or sim.verbosity_scripterrors,
                verified and '[D455-INFEED] label is UP; conveyor transfer approved' or
                    '[D455-INFEED] label orientation could not be verified'
            )
            if not verified then
                sim.setObjectOrientation(robot.box, sim.handle_world, {0.0, 0.0, 0.0})
            end
            transitionInfeed(robot, 'transfer_belt', now)
        end
    elseif robot.state == 'transfer_belt' then
        setTarget(robot, mix(robot.hoverStation, robot.hoverBelt, smoothstep(elapsed / 1.45)))
        if elapsed >= 1.45 and targetReached(robot, robot.hoverBelt, 0.13) then
            local boxPosition = sim.getObjectPosition(robot.box, sim.handle_world)
            local tipPosition = sim.getObjectPosition(robot.tip, sim.handle_world)
            local desiredCenter = {
                robot.beltPlace[1], robot.beltPlace[2], robot.beltSurface + boxHalfHeight + 0.005,
            }
            robot.beltPlaceTarget = {
                tipPosition[1] + desiredCenter[1] - boxPosition[1],
                tipPosition[2] + desiredCenter[2] - boxPosition[2],
                tipPosition[3] + desiredCenter[3] - boxPosition[3],
            }
            transitionInfeed(robot, 'lower_belt', now)
        end
    elseif robot.state == 'lower_belt' then
        setTarget(robot, mix(robot.hoverBelt, robot.beltPlaceTarget, smoothstep(elapsed / 0.8)))
        if elapsed >= 0.8 and targetReached(robot, robot.beltPlaceTarget, 0.09) then
            releaseInfeedBoxToBelt(robot, now)
            transitionInfeed(robot, 'raise_belt', now)
        end
    elseif robot.state == 'raise_belt' then
        setTarget(robot, mix(robot.beltPlaceTarget, robot.hoverBelt, smoothstep(elapsed / 0.75)))
        if elapsed >= 0.75 and targetReached(robot, robot.hoverBelt, 0.12) then
            transitionInfeed(robot, 'return_bin', now)
        end
    elseif robot.state == 'return_bin' then
        setTarget(robot, mix(robot.hoverBelt, robot.hoverBin, smoothstep(elapsed / 1.35)))
        if elapsed >= 1.35 and targetReached(robot, robot.hoverBin, 0.14) then
            transitionInfeed(robot, 'wait_bin', now)
        end
    elseif robot.state == 'recover_home' then
        setTarget(robot, mix(robot.recoveryStart, robot.hoverBin, smoothstep(elapsed / 1.8)))
        if elapsed >= 1.8 and targetReached(robot, robot.hoverBin, 0.15) then
            transitionInfeed(robot, 'wait_bin', now)
        end
    end

    if
        robot.state ~= 'wait_bin' and robot.state ~= 'recover_home' and
        now - robot.stateStarted >= motionWatchdogTimeout
    then
        infeedWatchdogCount = infeedWatchdogCount + 1
        sim.setInt32Signal('sorting_cell_infeed_watchdog_recoveries', infeedWatchdogCount)
        sim.addLog(sim.verbosity_scriptwarnings, '[UR10e-INFEED] motion timeout; safely resetting the cycle')
        if robot.box then
            stageSupplyBox(robot.box)
            robot.box = nil
        end
        sim.setInt32Signal('sorting_cell_infeed_vacuum', 0)
        robot.recoveryStart = sim.getObjectPosition(robot.tip, sim.handle_world)
        transitionInfeed(robot, 'recover_home', now)
    end
end

local function updateContinuousSupply(now)
    local inboundCount = 0
    for box, releasedAt in pairs(supplyActive) do
        local phase = supplyPhase[box]
        local p = sim.getObjectPosition(box, sim.handle_world)
        local insideBin =
            p[1] > -1.58 and p[1] < -0.72 and
            p[2] > -0.18 and p[2] < 0.66 and
            p[3] > 0.02 and p[3] < 0.82
        local escapedInfeed =
            p[3] < -0.20 or p[1] < -2.10 or p[1] > -0.20 or
            p[2] < -0.70 or p[2] > 2.35

        if phase == 'feeding' then
            if insideBin then
                supplyPhase[box] = 'bin'
                supplyLandedAt[box] = now
                phase = 'bin'
            elseif escapedInfeed or now - releasedAt >= 18.0 then
                stageSupplyBox(box)
                phase = 'staged'
            end
        elseif phase == 'bin' then
            if escapedInfeed then
                stageSupplyBox(box)
                phase = 'staged'
            elseif p[2] > 0.02 and math.floor(now * 8.0) % 2 == 0 then
                -- Model the short pulses of an industrial vibratory hopper.
                -- Gravity on the 18-degree insert and this small centering
                -- force keep parcels inside the reachable pick pocket.
                local centerForce = (-1.15 - p[1]) * 5.0
                sim.addForceAndTorque(box, {centerForce, -5.5, 0.0}, {0.0, 0.0, 0.0})
            end
        elseif phase == 'belt' then
            local beltTimedOut = supplyBeltAt[box] and now - supplyBeltAt[box] >= 75.0
            if p[3] < 0.15 or p[1] > 5.40 or beltTimedOut then
                sim.addLog(sim.verbosity_scriptwarnings, '[INFEED] conveyor parcel recovery cycle applied')
                stageSupplyBox(box)
                phase = 'staged'
            end
        elseif phase == 'sorted' then
            if supplySortedAt[box] and now - supplySortedAt[box] >= supplyRecycleDelay then
                stageSupplyBox(box)
                phase = 'staged'
            end
        end

        if phase == 'feeding' or phase == 'bin' then inboundCount = inboundCount + 1 end
    end

    if inboundCount >= 3 or now < nextSupplyRelease or #supplyBoxes == 0 then return end
    for offset = 0, #supplyBoxes - 1 do
        local index = ((supplyCursor + offset - 1) % #supplyBoxes) + 1
        local box = supplyBoxes[index]
        if supplyPhase[box] == 'staged' then
            releaseSupplyBox(box, now)
            supplyCursor = index % #supplyBoxes + 1
            nextSupplyRelease = now + supplyReleaseInterval
            return
        end
    end
    nextSupplyRelease = now + 0.25
end

local function startSafetyRecovery(robotIndex, now)
    local robot = robots[robotIndex]
    local tipPosition = sim.getObjectPosition(robot.tip, sim.handle_world)
    local holdTargets = {}
    for index, otherRobot in ipairs(robots) do
        holdTargets[index] = sim.getObjectPosition(otherRobot.tip, sim.handle_world)
    end
    safetyRecovery = {
        robotIndex = robotIndex,
        started = now,
        clearChecks = 0,
        holdTargets = holdTargets,
        liftTarget = {tipPosition[1], tipPosition[2], math.max(1.48, tipPosition[3] + 0.42)},
    }
    safetyRecoveryCount = safetyRecoveryCount + 1
    sim.setInt32Signal('sorting_cell_safety_collision', robotIndex)
    sim.setInt32Signal('sorting_cell_safety_recovery_active', 1)
    sim.setInt32Signal('sorting_cell_safety_recoveries', safetyRecoveryCount)
    sim.addLog(
        sim.verbosity_scriptwarnings,
        '[EMNIYET] ' .. robot.name .. ' temasi algilandi; bant durdu ve otomatik dikey geri cekilme basladi'
    )
end

local function finishSafetyRecovery(now)
    local recovery = safetyRecovery
    local robotIndex = recovery.robotIndex
    local robot = robots[robotIndex]
    local recoveryDuration = now - recovery.started
    for index, otherRobot in ipairs(robots) do
        if index ~= robotIndex then
            otherRobot.stateStarted = otherRobot.stateStarted + recoveryDuration
        end
    end

    if robotIndex == 1 then
        if robot.box then
            stageSupplyBox(robot.box)
            robot.box = nil
        end
        sim.setInt32Signal('sorting_cell_infeed_vacuum', 0)
        robot.recoveryStart = sim.getObjectPosition(robot.tip, sim.handle_world)
        transitionInfeed(robot, 'recover_home', now)
    else
        if robot.box and not heldBy[robot.box] then robot.box = nil end
        robot.recoveryStart = sim.getObjectPosition(robot.tip, sim.handle_world)
        robot.state = 'recover_home'
        robot.stateStarted = now
    end

    safetyRecovery = nil
    for index = 1, #collisionDebounce do collisionDebounce[index] = 0 end
    sim.setInt32Signal('sorting_cell_safety_collision', 0)
    sim.setInt32Signal('sorting_cell_safety_recovery_active', 0)
    sim.addLog(sim.verbosity_scriptinfos, '[EMNIYET] temas temizlendi; hucre otomatik devam ediyor')
end

local function updateSafetyRecovery(now)
    if not safetyRecovery then
        if controllerSteps % 10 ~= 0 then return false end
        local collisionIndex = collisionRobotIndex()
        for index = 1, #robots do
            if index == collisionIndex then
                collisionDebounce[index] = collisionDebounce[index] + 1
            else
                collisionDebounce[index] = 0
            end
        end
        if collisionIndex > 0 and collisionDebounce[collisionIndex] >= 3 then
            startSafetyRecovery(collisionIndex, now)
            return true
        end
        return false
    end

    local recovery = safetyRecovery
    local robot = robots[recovery.robotIndex]
    for index, otherRobot in ipairs(robots) do
        if index == recovery.robotIndex then
            setTarget(otherRobot, recovery.liftTarget)
        else
            setTarget(otherRobot, recovery.holdTargets[index])
        end
    end

    if now - recovery.started >= 2.5 then
        recovery.liftTarget[3] = math.max(recovery.liftTarget[3], 1.72)
    end
    if controllerSteps % 10 == 0 then
        if collisionRobotIndex() == 0 then
            recovery.clearChecks = recovery.clearChecks + 1
        else
            recovery.clearChecks = 0
        end
    end
    if recovery.clearChecks >= 6 and targetReached(robot, recovery.liftTarget, 0.18) then
        finishSafetyRecovery(now)
        return false
    end
    return true
end

function sysCall_init()
    sim.setInt32Signal('sorting_cell_controller_state', 1)
    selfHandle = sim.getObject('.')
    cellRoot = references('cell.root')[1]
    beltSlats = references('belt.slats')
    beltCollisionShapes = references('belt.collision')
    redBoxes = references('boxes.red')
    blueBoxes = references('boxes.blue')
    supplyBoxes = references('boxes.supply')
    supplySpawn = references('supply.spawn')[1]
    supplyStaging = references('supply.staging')[1]
    supplyChute = references('supply.chute')[1]

    beltSpeed = 0.15
    beltLength = 5.0
    beltMinimumX = 0.0
    beltDrop = 0.18
    beltPathLength = 2.0 * beltLength + 2.0 * beltDrop
    beltTopZ = sim.getObjectPosition(beltSlats[1], sim.handle_world)[3]
    beltPhases = {}
    for index = 1, #beltSlats do
        beltPhases[index] = (index - 1) * beltPathLength / #beltSlats
    end
    cameraDecisionX = 1.58
    boxHalfHeight = 0.09
    motionWatchdogTimeout = 40.0

    boxRoutes = {}
    classified = {}
    heldBy = {}
    completed = {}
    for _, box in ipairs(redBoxes) do boxRoutes[box] = 1 end
    for _, box in ipairs(blueBoxes) do boxRoutes[box] = 2 end

    supplyReleaseInterval = sim.getFloatSignal('sorting_cell_supply_release_interval') or 3.2
    supplyRecycleDelay = sim.getFloatSignal('sorting_cell_supply_recycle_delay') or 10.0
    supplyActive = {}
    supplyLandedAt = {}
    supplyBeltAt = {}
    supplySortedAt = {}
    supplyPhase = {}
    supplyRoute = {}
    supplyIndex = {}
    supplyCursor = 1
    nextSupplyRelease = 0.25
    infeedCompletedCount = 0
    orientationCorrectionCount = 0
    supplyCompletedCount = 0
    for index, box in ipairs(supplyBoxes) do
        supplyIndex[box] = index
        supplyRoute[box] = index % 2 == 1 and 1 or 2
        supplyPhase[box] = 'staged'
        stageSupplyBox(box)
    end

    robots = {
        {
            name = 'UR10e-INFEED',
            base = references('robot.1.base')[1],
            tip = references('robot.1.tip')[1],
            target = references('robot.1.target')[1],
            joints = references('robot.1.joints'),
            hoverBin = {-1.10, 0.00, 1.42},
            hoverStation = {-0.45, 0.00, 1.42},
            stationCenter = {-0.45, 0.00},
            stationSurface = 0.903,
            hoverBelt = {0.38, 0.00, 1.43},
            beltPlace = {0.38, 0.00},
            beltSurface = beltTopZ + 0.013,
            state = 'wait_bin',
            stateStarted = 0.0,
        },
        {
            name = 'UR10e-KIRMIZI',
            base = references('robot.2.base')[1],
            tip = references('robot.2.tip')[1],
            target = references('robot.2.target')[1],
            joints = references('robot.2.joints'),
            route = 1,
            pick = {3.10, 0.00, 1.02},
            hoverPick = {3.10, 0.00, 1.55},
            hoverDrop = {2.85, -1.70, 1.48},
            drop = {2.85, -1.70, 0.92},
            pickTolerance = 0.36,
            state = 'wait',
            stateStarted = 0.0,
        },
        {
            name = 'UR10e-MAVI',
            base = references('robot.3.base')[1],
            tip = references('robot.3.tip')[1],
            target = references('robot.3.target')[1],
            joints = references('robot.3.joints'),
            route = 2,
            pick = {4.35, 0.00, 1.02},
            hoverPick = {4.35, 0.00, 1.55},
            hoverDrop = {4.30, 1.70, 1.48},
            drop = {4.30, 1.70, 0.92},
            pickTolerance = 0.36,
            state = 'wait',
            stateStarted = 0.0,
        },
    }

    collisionDebounce = {0, 0, 0}
    safetyRecovery = nil
    safetyRecoveryCount = 0
    infeedWatchdogCount = 0
    sorterWatchdogCount = 0
    lastProgressValue = 0
    lastProgressAt = 0.0

    conveyorCollection = sim.createCollection(0)
    for _, shape in ipairs(beltCollisionShapes) do
        sim.addItemToCollection(conveyorCollection, sim.handle_single, shape, 0)
    end
    for _, robot in ipairs(robots) do
        configureIk(robot)
        robot.collisionCollection = sim.createCollection(0)
        -- Snapshot only the robot's own shapes. A parcel later parented to the
        -- vacuum TCP must not become part of the robot-vs-belt safety query.
        for _, shape in ipairs(sim.getObjectsInTree(robot.base, sim.object_shape_type, 0)) do
            sim.addItemToCollection(robot.collisionCollection, sim.handle_single, shape, 0)
        end
    end
    controllerSteps = 0
    sim.setInt32Signal('sorting_cell_safety_collision', 0)
    sim.setInt32Signal('sorting_cell_safety_recovery_active', 0)
    sim.setInt32Signal('sorting_cell_safety_recoveries', 0)
    sim.setInt32Signal('sorting_cell_infeed_watchdog_recoveries', 0)
    sim.setInt32Signal('sorting_cell_sorter_watchdog_recoveries', 0)
    sim.setInt32Signal('sorting_cell_heartbeat_ms', 0)
    sim.setInt32Signal('sorting_cell_progress_stalled', 0)
    sim.setInt32Signal('sorting_cell_infeed_vacuum', 0)
    sim.setInt32Signal('sorting_cell_infeed_completed', 0)
    sim.setInt32Signal('sorting_cell_orientation_corrections', 0)
    sim.setInt32Signal('sorting_cell_supply_completed', 0)
    sim.setStringSignal('sorting_cell_infeed_state', 'wait_bin')
    sim.setInt32Signal('sorting_cell_controller_state', 2)
    sim.addLog(
        sim.verbosity_scriptinfos,
        '[CELL] end-to-end physical parcel flow, label orientation, and color sorting active'
    )
end

function sysCall_actuation()
    controllerSteps = controllerSteps + 1
    if controllerSteps % 100 == 0 then
        sim.setInt32Signal('sorting_cell_controller_steps', controllerSteps)
        sim.setInt32Signal('sorting_cell_heartbeat_ms', math.floor(sim.getSimulationTime() * 1000.0))
    end
    local now = sim.getSimulationTime()
    local progressValue = infeedCompletedCount + supplyCompletedCount
    if progressValue ~= lastProgressValue then
        lastProgressValue = progressValue
        lastProgressAt = now
        sim.setInt32Signal('sorting_cell_progress_stalled', 0)
    elseif now - lastProgressAt >= 60.0 then
        sim.setInt32Signal('sorting_cell_progress_stalled', 1)
    end
    local recovering = updateSafetyRecovery(now)
    if not recovering then
        updateBelt(sim.getSimulationTimeStep())
        updateContinuousSupply(now)
        updateCameraDecisions()
        updateInfeed(robots[1], now)
        updateSorter(robots[2], now)
        updateSorter(robots[3], now)
    end
    if controllerSteps % 4 == 0 then
        for index, robot in ipairs(robots) do driveIk(robot, index) end
    end
end

function sysCall_cleanup()
    if robots then
        for _, robot in ipairs(robots) do
            if robot.ikEnvironment then simIK.eraseEnvironment(robot.ikEnvironment) end
            if robot.collisionCollection then sim.destroyCollection(robot.collisionCollection) end
        end
    end
    if conveyorCollection then sim.destroyCollection(conveyorCollection) end
end

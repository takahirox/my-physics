export const RACE_PHASE = Object.freeze({
  COUNTDOWN: 'countdown',
  RACING: 'racing',
  FINISHED: 'finished',
});

const DEFAULT_CHECKPOINTS = Object.freeze([0.25, 0.5, 0.75, 1]);
const PROGRESS_EPSILON = 1e-9;

function finite(value, fallback = 0) {
  return Number.isFinite(value) ? value : fallback;
}

function normalizedProgress(value) {
  const finiteValue = finite(value);
  return ((finiteValue % 1) + 1) % 1;
}

function gridDistance(progress) {
  const normalized = normalizedProgress(progress);
  return normalized > 0.8 ? normalized - 1 : normalized;
}

function copyParticipant(participant) {
  return {
    ...participant,
    lapTimesS: [...participant.lapTimesS],
  };
}

function makeParticipant(index, progress, raceStartTimeS) {
  const rawProgress = normalizedProgress(progress);
  return {
    index,
    rawProgress,
    distanceLaps: gridDistance(rawProgress),
    completedLaps: 0,
    nextCheckpoint: 0,
    currentLapStartedAtS: raceStartTimeS,
    lapTimesS: [],
    bestLapTimeS: null,
    finishTimeS: null,
    finishPosition: null,
    invalidProgressSamples: 0,
  };
}

/**
 * Deterministic application-layer race state.
 *
 * The caller supplies authoritative PhysicsWorld time and track progress. The
 * director never changes vehicle state, forces, inputs, or physical results.
 * Quarter-lap checkpoints and bounded progress deltas prevent reverse line
 * crossings and discontinuous nearest-segment jumps from awarding a lap.
 */
export class RaceDirector {
  constructor({
    totalLaps = 3,
    countdownDurationS = 3,
    checkpoints = DEFAULT_CHECKPOINTS,
    maximumProgressDelta = 0.08,
    playerIndex = 0,
  } = {}) {
    if (!Number.isInteger(totalLaps) || totalLaps < 1) throw new RangeError('totalLaps must be a positive integer');
    if (!Number.isFinite(countdownDurationS) || countdownDurationS < 0) {
      throw new RangeError('countdownDurationS must be non-negative');
    }
    if (!Number.isFinite(maximumProgressDelta) || maximumProgressDelta <= 0 || maximumProgressDelta >= 0.5) {
      throw new RangeError('maximumProgressDelta must be between zero and one half-lap');
    }
    const orderedCheckpoints = [...checkpoints];
    if (
      orderedCheckpoints.length === 0
      || orderedCheckpoints.some((checkpoint, index) => (
        !Number.isFinite(checkpoint)
        || checkpoint <= 0
        || checkpoint > 1
        || (index > 0 && checkpoint <= orderedCheckpoints[index - 1])
      ))
      || orderedCheckpoints.at(-1) !== 1
    ) {
      throw new RangeError('checkpoints must be strictly ordered fractions ending at 1');
    }
    this.config = Object.freeze({
      totalLaps,
      countdownDurationS,
      checkpoints: Object.freeze(orderedCheckpoints),
      maximumProgressDelta,
      playerIndex,
    });
    this.reset(0, []);
  }

  reset(physicsTimeS = 0, progresses = []) {
    const now = finite(physicsTimeS);
    this.phase = RACE_PHASE.COUNTDOWN;
    this.resetTimeS = now;
    this.raceStartTimeS = now + this.config.countdownDurationS;
    this.finishOrder = [];
    this.participants = [...progresses].map((progress, index) => makeParticipant(index, progress, this.raceStartTimeS));
    this.lastPhysicsTimeS = now;
    return this.view(now);
  }

  ensureParticipants(progresses) {
    while (this.participants.length < progresses.length) {
      const index = this.participants.length;
      this.participants.push(makeParticipant(index, progresses[index], this.raceStartTimeS));
    }
    if (this.participants.length > progresses.length) this.participants.length = progresses.length;
  }

  update(physicsTimeS, progresses) {
    const now = finite(physicsTimeS, this.lastPhysicsTimeS);
    if (now < this.lastPhysicsTimeS) throw new RangeError('physics time must be monotonic');
    if (!Array.isArray(progresses)) throw new TypeError('progresses must be an array');
    this.ensureParticipants(progresses);
    const events = [];

    if (this.phase === RACE_PHASE.COUNTDOWN) {
      for (const participant of this.participants) {
        participant.rawProgress = normalizedProgress(progresses[participant.index]);
      }
      if (now >= this.raceStartTimeS) {
        this.phase = RACE_PHASE.RACING;
        for (const participant of this.participants) {
          participant.currentLapStartedAtS = this.raceStartTimeS;
        }
        events.push({ type: 'race-started', timeS: this.raceStartTimeS });
      }
    }

    if (this.phase === RACE_PHASE.RACING) {
      for (const participant of this.participants) {
        this.updateParticipant(participant, progresses[participant.index], now, events);
      }
      const player = this.participants[this.config.playerIndex];
      if (player?.finishTimeS !== null) {
        this.phase = RACE_PHASE.FINISHED;
        events.push({ type: 'race-finished', timeS: player.finishTimeS });
      }
    }

    this.lastPhysicsTimeS = now;
    return { ...this.view(now), events };
  }

  updateParticipant(participant, progress, now, events) {
    if (participant.finishTimeS !== null) return;
    const rawProgress = normalizedProgress(progress);
    let delta = rawProgress - participant.rawProgress;
    if (delta > 0.5) delta -= 1;
    if (delta < -0.5) delta += 1;
    participant.rawProgress = rawProgress;

    if (Math.abs(delta) > this.config.maximumProgressDelta) {
      participant.invalidProgressSamples += 1;
      events.push({ type: 'progress-rejected', vehicle: participant.index, delta });
      return;
    }

    const previousDistance = participant.distanceLaps;
    participant.distanceLaps += delta;
    if (delta <= 0) return;

    let checkpointDistance = this.nextCheckpointDistance(participant);
    while (
      previousDistance < checkpointDistance - PROGRESS_EPSILON
      && checkpointDistance <= participant.distanceLaps + PROGRESS_EPSILON
    ) {
      const checkpoint = this.config.checkpoints[
        participant.nextCheckpoint % this.config.checkpoints.length
      ];
      participant.nextCheckpoint += 1;
      events.push({
        type: checkpoint === 1 ? 'lap-completed' : 'checkpoint',
        vehicle: participant.index,
        checkpoint,
        timeS: now,
      });
      if (checkpoint === 1) this.completeLap(participant, now, events);
      if (participant.finishTimeS !== null) break;
      checkpointDistance = this.nextCheckpointDistance(participant);
    }
  }

  nextCheckpointDistance(participant) {
    const checkpointIndex = participant.nextCheckpoint % this.config.checkpoints.length;
    const lapIndex = Math.floor(participant.nextCheckpoint / this.config.checkpoints.length);
    return lapIndex + this.config.checkpoints[checkpointIndex];
  }

  completeLap(participant, now, events) {
    participant.completedLaps += 1;
    const lapTimeS = Math.max(0, now - participant.currentLapStartedAtS);
    participant.lapTimesS.push(lapTimeS);
    participant.bestLapTimeS = participant.bestLapTimeS === null
      ? lapTimeS
      : Math.min(participant.bestLapTimeS, lapTimeS);
    participant.currentLapStartedAtS = now;
    events.at(-1).lap = participant.completedLaps;
    events.at(-1).lapTimeS = lapTimeS;

    if (participant.completedLaps >= this.config.totalLaps) {
      participant.finishTimeS = Math.max(0, now - this.raceStartTimeS);
      this.finishOrder.push(participant.index);
      participant.finishPosition = this.finishOrder.length;
      events.push({
        type: 'vehicle-finished',
        vehicle: participant.index,
        position: participant.finishPosition,
        timeS: participant.finishTimeS,
      });
    }
  }

  standings() {
    return this.participants
      .map(copyParticipant)
      .sort((a, b) => {
        if (a.finishPosition !== null || b.finishPosition !== null) {
          if (a.finishPosition === null) return 1;
          if (b.finishPosition === null) return -1;
          return a.finishPosition - b.finishPosition;
        }
        return b.distanceLaps - a.distanceLaps || a.index - b.index;
      })
      .map((participant, index) => ({ ...participant, position: index + 1 }));
  }

  view(physicsTimeS = this.lastPhysicsTimeS) {
    const now = finite(physicsTimeS, this.lastPhysicsTimeS);
    const standings = this.standings();
    const player = standings.find(({ index }) => index === this.config.playerIndex) ?? null;
    const countdownRemainingS = this.phase === RACE_PHASE.COUNTDOWN
      ? Math.max(0, this.raceStartTimeS - now)
      : 0;
    const raceTimeS = this.phase === RACE_PHASE.COUNTDOWN
      ? 0
      : player?.finishTimeS ?? Math.max(0, now - this.raceStartTimeS);
    return {
      phase: this.phase,
      totalLaps: this.config.totalLaps,
      countdownRemainingS,
      countdownValue: countdownRemainingS > 0 ? Math.ceil(countdownRemainingS) : 0,
      raceTimeS,
      player,
      standings,
    };
  }

  snapshot() {
    return {
      version: 1,
      config: {
        ...this.config,
        checkpoints: [...this.config.checkpoints],
      },
      phase: this.phase,
      resetTimeS: this.resetTimeS,
      raceStartTimeS: this.raceStartTimeS,
      finishOrder: [...this.finishOrder],
      participants: this.participants.map(copyParticipant),
      lastPhysicsTimeS: this.lastPhysicsTimeS,
    };
  }

  restore(snapshot) {
    if (snapshot?.version !== 1) throw new TypeError('unsupported race snapshot');
    if (JSON.stringify(snapshot.config) !== JSON.stringify({ ...this.config, checkpoints: [...this.config.checkpoints] })) {
      throw new TypeError('race snapshot configuration mismatch');
    }
    if (!Object.values(RACE_PHASE).includes(snapshot.phase)) throw new TypeError('invalid race phase');
    this.phase = snapshot.phase;
    this.resetTimeS = snapshot.resetTimeS;
    this.raceStartTimeS = snapshot.raceStartTimeS;
    this.finishOrder = [...snapshot.finishOrder];
    this.participants = snapshot.participants.map(copyParticipant);
    this.lastPhysicsTimeS = snapshot.lastPhysicsTimeS;
    return this.view();
  }
}

export function formatRaceTime(timeS) {
  if (!Number.isFinite(timeS)) return '—';
  const milliseconds = Math.max(0, Math.round(timeS * 1000));
  const minutes = Math.floor(milliseconds / 60_000);
  const seconds = Math.floor(milliseconds / 1000) % 60;
  const remainder = milliseconds % 1000;
  return `${minutes}:${String(seconds).padStart(2, '0')}.${String(remainder).padStart(3, '0')}`;
}

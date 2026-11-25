//! Owning sync-fence helpers for AHB slot handoffs.

use crate::{IpaError, IpaResult};
use cam_types::{AhbFenceTokens, SyncFenceToken, INVALID_SYNC_FD};
use std::os::fd::{AsRawFd, FromRawFd, IntoRawFd, OwnedFd, RawFd};

/// Owns a sync fd returned by camera, `AHardwareBuffer_unlock`, or Vulkan.
///
/// A [`SyncFenceToken`] is only a non-owning view of this fd. The owner stays
/// alive for the whole inference call that receives the token.
#[derive(Debug)]
pub struct SyncFence {
    fd: OwnedFd,
    generation: u64,
}

impl SyncFence {
    /// Take ownership of `fd`. Negative fds are not sync fences.
    pub fn from_raw_fd(fd: RawFd, generation: u64) -> IpaResult<Self> {
        if fd < 0 {
            return Err(IpaError::Buffer(format!(
                "invalid sync fd {fd} for generation {generation}"
            )));
        }
        // SAFETY: the caller transfers ownership of this fd.
        Ok(Self {
            fd: unsafe { OwnedFd::from_raw_fd(fd) },
            generation,
        })
    }

    pub fn as_raw_fd(&self) -> RawFd {
        self.fd.as_raw_fd()
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn token(&self) -> SyncFenceToken {
        SyncFenceToken::new(self.as_raw_fd(), self.generation)
    }

    /// Transfer ownership back to the caller, e.g. to queue the fd to Android.
    pub fn into_raw_fd(self) -> RawFd {
        self.fd.into_raw_fd()
    }
}

/// Current fence state for one AHB slot.
///
/// `begin()` starts a new generation and takes ownership of the camera acquire
/// fence. `complete()` records the release fence produced by CPU unlock, GPU
/// submit, or display queue. A slot may be reused only after all consumers have
/// waited on the release fence.
#[derive(Debug, Default)]
pub struct AhbFenceState {
    generation: u64,
    acquire: Option<SyncFence>,
    release: Option<SyncFence>,
}

impl AhbFenceState {
    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn tokens(&self) -> AhbFenceTokens {
        AhbFenceTokens::new(
            self.acquire
                .as_ref()
                .map_or(INVALID_SYNC_FD, SyncFence::as_raw_fd),
            self.release
                .as_ref()
                .map_or(INVALID_SYNC_FD, SyncFence::as_raw_fd),
            self.generation,
        )
    }

    /// Start a new capture generation. `acquire_fd` is transferred to this
    /// state and closed when the slot advances again or is dropped. A pending
    /// release fence must be taken by the display path first, so an unwaited
    /// slot cannot be recycled.
    pub fn begin(&mut self, acquire_fd: RawFd) -> IpaResult<u64> {
        if self.release.is_some() {
            return Err(IpaError::Buffer(format!(
                "AHB generation {} release fence was not taken before reuse",
                self.generation
            )));
        }
        let generation = self
            .generation
            .checked_add(1)
            .ok_or_else(|| IpaError::Buffer("AHB fence generation counter overflow".to_string()))?;
        let acquire = if acquire_fd < 0 {
            None
        } else {
            Some(SyncFence::from_raw_fd(acquire_fd, generation)?)
        };
        self.acquire = acquire;
        self.release = None;
        self.generation = generation;
        Ok(generation)
    }

    /// Record the release fd for the current generation.
    pub fn complete(&mut self, release_fd: RawFd) -> IpaResult<()> {
        let release = if release_fd < 0 {
            None
        } else {
            Some(SyncFence::from_raw_fd(release_fd, self.generation)?)
        };
        self.release = release;
        Ok(())
    }

    /// Take the current release fd for display queueing. The caller owns it.
    pub fn take_release(&mut self) -> Option<SyncFence> {
        self.release.take()
    }

    /// Reject a binding from a different generation or with mismatched fds.
    pub fn validate(&self, tokens: AhbFenceTokens) -> IpaResult<()> {
        if tokens.generation != self.generation {
            return Err(IpaError::Buffer(format!(
                "AHB generation mismatch: prepared {}, frame {}",
                self.generation, tokens.generation
            )));
        }
        let expected = self.tokens();
        if tokens.acquire.fd != expected.acquire.fd || tokens.release.fd != expected.release.fd {
            return Err(IpaError::Buffer(format!(
                "AHB fence mismatch for generation {}: expected {}/{}, got {}/{}",
                self.generation,
                expected.acquire.fd,
                expected.release.fd,
                tokens.acquire.fd,
                tokens.release.fd
            )));
        }
        Ok(())
    }
}

/// Whether a buffer must be zeroed before its first read.
///
/// Skipping the clear is only valid when the producer overwrites every byte
/// before any consumer reads it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ZeroPolicy {
    /// Zero the allocation before exposing it to the pipeline.
    #[default]
    NeedZero,
    /// Producer commits to a full overwrite; a future allocator may skip clear.
    FullOverwrite,
}

impl ZeroPolicy {
    pub fn needs_zero(self) -> bool {
        matches!(self, Self::NeedZero)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pipe_fds() -> (RawFd, RawFd) {
        let mut fds = [-1, -1];
        assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0);
        (fds[0], fds[1])
    }

    fn fd_is_open(fd: RawFd) -> bool {
        unsafe { libc::fcntl(fd, libc::F_GETFD) >= 0 }
    }

    #[test]
    fn sync_fence_owns_fd() {
        let (read_fd, write_fd) = pipe_fds();
        {
            let fence = SyncFence::from_raw_fd(read_fd, 4).expect("fence");
            assert_eq!(fence.generation(), 4);
            assert!(fence.token().is_valid());
            assert!(fd_is_open(read_fd));
        }
        assert!(!fd_is_open(read_fd));
        unsafe { libc::close(write_fd) };
    }

    #[test]
    fn fence_state_tracks_generation_and_release() {
        let (acquire, acquire_w) = pipe_fds();
        let (release, release_w) = pipe_fds();
        let mut state = AhbFenceState::default();
        assert_eq!(state.begin(acquire).unwrap(), 1);
        state.complete(release).unwrap();

        let current = state.tokens();
        state.validate(current).unwrap();
        assert_eq!(current.generation, 1);
        assert_eq!(current.acquire.fd, acquire);
        assert_eq!(current.release.fd, release);

        let stale = cam_types::AhbFenceTokens::new(acquire, release, 0);
        assert!(state.validate(stale).is_err());

        let taken = state.take_release().expect("release fence");
        assert_eq!(taken.into_raw_fd(), release);
        unsafe {
            libc::close(acquire_w);
            libc::close(release_w);
        }
    }

    #[test]
    fn slot_cannot_be_reused_before_release_is_taken() {
        let (acquire, acquire_w) = pipe_fds();
        let (release, release_w) = pipe_fds();
        let (next_acquire, next_acquire_w) = pipe_fds();
        let mut state = AhbFenceState::default();
        assert_eq!(state.begin(acquire).unwrap(), 1);
        state.complete(release).unwrap();
        assert!(state.begin(next_acquire).is_err());

        let taken = state.take_release().expect("release fence");
        assert_eq!(taken.into_raw_fd(), release);
        assert_eq!(state.begin(next_acquire).unwrap(), 2);
        assert_eq!(state.tokens().generation, 2);
        unsafe {
            libc::close(acquire_w);
            libc::close(release_w);
            libc::close(next_acquire_w);
        }
    }

    #[test]
    fn cancel_drain_is_idempotent_for_single_and_pair_tokens() {
        let (acquire, acquire_w) = pipe_fds();
        let (release, release_w) = pipe_fds();
        let mut state = AhbFenceState::default();
        let generation = state.begin(acquire).expect("begin");
        state.complete(release).expect("complete");

        let pair = state.tokens();
        let single: SyncFenceToken = pair.acquire;
        assert!(single.is_valid());
        assert!(single.matches(generation));
        assert!(pair.matches(generation));
        state.validate(pair).expect("live pair");

        let taken = state.take_release().expect("release fence");
        assert_eq!(taken.as_raw_fd(), release);
        assert!(state.take_release().is_none());
        let drained = state.tokens();
        assert!(!drained.release.is_valid());
        assert!(state.validate(pair).is_err());

        let release_fd = taken.into_raw_fd();
        assert!(fd_is_open(release_fd));
        unsafe { libc::close(release_fd) };
        assert!(!fd_is_open(release_fd));
        unsafe {
            libc::close(acquire_w);
            libc::close(release_w);
        }

        let (next_acquire, next_acquire_w) = pipe_fds();
        assert_eq!(
            state.begin(next_acquire).expect("re-acquire"),
            generation + 1
        );
        assert!(!single.matches(generation + 1));
        assert!(!pair.matches(generation + 1));
        unsafe { libc::close(next_acquire_w) };
    }

    #[test]
    fn zero_policy_defaults_to_safe_clear() {
        assert!(ZeroPolicy::default().needs_zero());
        assert!(!ZeroPolicy::FullOverwrite.needs_zero());
    }
}

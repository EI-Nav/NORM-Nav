window.HELP_IMPROVE_VIDEOJS = false;

document.addEventListener('DOMContentLoaded', function () {
  var carouselOptions = {
    slidesToScroll: 1,
    slidesToShow: 1,
    loop: true,
    infinite: true,
    autoplay: true,
    autoplaySpeed: 5000
  };

  if (typeof bulmaCarousel !== 'undefined' && typeof bulmaCarousel.attach === 'function') {
    bulmaCarousel.attach('.carousel', carouselOptions);
  }
  if (typeof bulmaSlider !== 'undefined' && typeof bulmaSlider.attach === 'function') {
    bulmaSlider.attach();
  }

  // Fade-in on scroll
  var fadeElements = document.querySelectorAll('.fade-in');
  if (fadeElements.length && 'IntersectionObserver' in window) {
    var observer = new IntersectionObserver(function (entries) {
      entries.forEach(function (entry) {
        if (entry.isIntersecting) {
          entry.target.classList.add('visible');
          entry.target.classList.remove('not-visible');
        }
      });
    }, { root: null, rootMargin: '0px', threshold: 0.1 });

    fadeElements.forEach(function (el) {
      var rect = el.getBoundingClientRect();
      var inViewport = rect.top < window.innerHeight && rect.bottom > 0;
      if (!inViewport) {
        el.classList.add('not-visible');
      }
      observer.observe(el);
    });
  }

  // Copy-to-clipboard buttons (e.g. BibTeX)
  function fallbackCopy(text, onDone) {
    var ta = document.createElement('textarea');
    ta.value = text;
    ta.setAttribute('readonly', '');
    ta.style.position = 'fixed';
    ta.style.opacity = '0';
    document.body.appendChild(ta);
    ta.select();
    try {
      document.execCommand('copy');
      if (onDone) onDone();
    } catch (err) {
      console.warn('Copy failed:', err);
    }
    document.body.removeChild(ta);
  }

  document.querySelectorAll('.copy-btn').forEach(function (btn) {
    btn.addEventListener('click', function () {
      var targetId = btn.getAttribute('data-target');
      if (!targetId) return;
      var target = document.getElementById(targetId);
      if (!target) return;
      var text = target.innerText.trim();

      var label = btn.querySelector('span');
      var originalLabel = label ? label.textContent : null;

      var markCopied = function () {
        btn.classList.add('is-copied');
        if (label) label.textContent = 'Copied!';
        setTimeout(function () {
          btn.classList.remove('is-copied');
          if (label && originalLabel !== null) label.textContent = originalLabel;
        }, 1800);
      };

      if (navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(text).then(markCopied).catch(function () {
          fallbackCopy(text, markCopied);
        });
      } else {
        fallbackCopy(text, markCopied);
      }
    });
  });

  // Floating back-to-top button
  var backToTop = document.querySelector('.back-to-top');
  if (backToTop) {
    var toggleVisibility = function () {
      if (window.scrollY > 600) {
        backToTop.classList.add('is-visible');
      } else {
        backToTop.classList.remove('is-visible');
      }
    };
    window.addEventListener('scroll', toggleVisibility, { passive: true });
    toggleVisibility();

    backToTop.addEventListener('click', function () {
      var prefersReduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
      window.scrollTo({ top: 0, behavior: prefersReduced ? 'auto' : 'smooth' });
    });
  }
});

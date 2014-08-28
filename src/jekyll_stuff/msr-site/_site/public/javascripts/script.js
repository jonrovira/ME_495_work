


$(document).ready(function() {

	var totalWidth = 0;

	$('main#home section#projects div#slider-wrapper ul li').each(function() {
		totalWidth += $(this).outerWidth(true);
	});

	var maxScrollPosition = totalWidth - $('main#home section#projects div#slider-wrapper').outerWidth();

	var toGalleryItem = function($target) {
		if($target.length) {

			var newPosition = $target.position().left;

			if(newPosition <= maxScrollPosition) {
				$target.addClass('active');
				$target.siblings().removeClass('active');

				$('main#home section#projects div#slider-wrapper ul').animate({
					left : - newPosition
				});
			}
			else {
				$('main#home section#projects div#slider-wrapper ul').animate({
					left : -maxScrollPosition
				});
			}
		}
	};

	$('main#home section#projects div#slider-wrapper ul').width(totalWidth);
	$('main#home section#projects div#slider-wrapper ul li:first').addClass('active');
	$('main#home section#projects div#slider-wrapper div#slider-controls div#slider-prev').click(function() {
		var $target = $('main#home section#projects div#slider-wrapper ul li.active').prev();
		toGalleryItem($target);
	});
	$('main#home section#projects div#slider-wrapper div#slider-controls div#slider-next').click(function() {
		var $target = $('main#home section#projects div#slider-wrapper ul li.active').next();
		toGalleryItem($target);
	});

	console.log(totalWidth);


});

/*
 * Functions
 */

// Initializes slider dimensions and state
var initializeSliderState = function() {

	// Calculate and set inner slider width
	var totalWidth = 0;
	$('div#slider-wrapper ul li').each(function() {
		totalWidth += $(this).outerWidth(true);
	});
	maxScrollPosition = totalWidth - $('div#slider-wrapper').outerWidth();
	$('div#slider-wrapper ul').width(totalWidth);

	// Initialize first slider item as the active one
	$('main#home section#recent-projects div#slider-wrapper ul li:first').addClass('active');
	setSliderText();

	return;
};
// Controls slider animation on home page
var toGalleryItem = function($target, maxScrollPosition) {
	if($target.length) {

		var newPosition = $target.position().left;

		if(newPosition <= maxScrollPosition) {
			$target.addClass('active');
			$target.siblings().removeClass('active');

			$('main#home section#recent-projects div#slider-wrapper ul').animate({
				left : - newPosition
			});
		}
		else {
			$('main#home section#recent-projects div#slider-wrapper ul').animate({
				left : -maxScrollPosition
			});
		}
		setSliderText();
	}

	return;
};
// Sets slider subheader text
var setSliderText = function() {
	$('div#slider-info h2').html($('div#slider-wrapper ul li.active a h2').html());
	$('div#slider-info p').html($('div#slider-wrapper ul li.active a p').html());

	return;
};
// Initialize class select buttons on Students page
var initializeClassSelectButtons = function() {

	// Global (ish)
	var $active;

	// User did not select specific class year
	if( !checkURLForString('class_year') ) {
		$active = $('ul#class-select-list li:first');
	}
	// User did select specific class year
	else {
		var classYear = window.location.search.replace('?class_year=', '');
		$('ul#class-select-list li').each(function() {
			if( $(this).children('button').val() == classYear ) {
				$active = $(this);
			}
		});
	}

	// Change button css and show correct students
	$active.addClass('active');
	showStudents($active.children('button'));

	return;
};
// Show selected students on Students page
var showStudents = function($selection) {

	// Add active class for button styling
	$selection.parent().siblings().removeClass('active');
	$selection.parent().addClass('active');

	// Show correct students
	if( $selection.val() == "all" ) {
		$('section#students-list ul li').show();
	}
	else {
		$('section#students-list ul li').hide();
		$('section#students-list ul li.class-year-' + $selection.val()).show();
	}

	return;
};
// Check URL for a string
var checkURLForString = function(s) {
	if( window.location.href.indexOf(s) > -1 ) {
	       return true;
	}

	return;
};
// Edit URL text
var editURLText = function($selection) {
	var current = window.location.pathname;
	var newHref = current.replace('students/', 'students/?class_year=' + $selection.val());
	history.replaceState('data', '', newHref);

	return;
};
// Alphabetize a list of strings
var alphabetizeList = function($list) {
	var unalphabetized = [];
	var alphabetized = [];

	// Get list of unalphabetized strings
	$list.children('li').each(function() {
		unalphabetized.push($(this).html());
	});

	// Alphabetize that list
	alphabetized = unalphabetized.sort();

	// Replace the DOM html with the alphabetized slist
	var count = 0;
	$list.children('li').each(function() {
		$(this).html(alphabetized[count]);
		count += 1;
	});

	// Show the previously hidden list
	$list.show();
};
// Check which page is currently loaded
var checkWhichPage = function() {
	var mainId = $('main').attr('id');
	return mainId;
};
// Toggle the color of the tag element
var toggleTag = function($tag) {
	// If the All tag wasn't clicked
	if( $tag.attr('id') != 'all-tag' ) {
		// Don't keep All tag clicked
		$('section#tags-list ul li#all-tag').removeClass('clicked');

		// Toggle clicked
		$tag.toggleClass('clicked');

		// If no tags are clicked now, click All tag
		if( $('section#tags-list ul li.clicked').length == 0 ) {
			$('section#tags-list ul li#all-tag').addClass('clicked');
		}
		return;
	}
	// Click all tag
	$('section#tags-list ul li#all-tag').addClass('clicked');
	$('section#tags-list ul li:not(#all-tag)').removeClass('clicked');
	return;
};
// Logic for showing resource entries based on tag selection
var filterByTag = function($tag) {
	// Undo hide class
	$('section#resources-list > ul > li').removeClass('hidden-by-tags');

	var clickedList = [];

	// Get list of selected tags
	$('section#tags-list ul li.clicked').each(function() {
		clickedList.push($(this).html());
	});
	
	// If All tag is selected, show all and return
	if(clickedList[0] == 'All') {
		$('section#resources-list > ul > li').removeClass('hidden-by-tags');
		return;
	}

	// Cycle through each resource
	$('section#resources-list > ul > li').each(function() {

		
		// By default, hide resource
		var show = false;

		// Cycle through each resource's tags
		$(this).find('li.resource-tag').each(function() {

			// If there's one that matches a selected tag, we need to show the resource
			if( clickedList.indexOf($(this).html()) > -1 ) {
				show = true;
			}
		});

		// Show the resource if deemed necessary
		if( !show ) {
			$(this).addClass('hidden-by-tags');
		}
	});
};
// Search for resources from search bar input
var searchResources = function(search) {
	// Undo hide class
	$('section#resources-list > ul > li').removeClass('hidden-by-search');

	// If search is valid
	if( search.length > 0 ) {
		// Cycle through each resource
		$('section#resources-list > ul > li').each(function() {

			var $resource = $(this);

			// If resource title doesn't contain string
			if( $resource.find('h1.resource-title').html().toLowerCase().indexOf(search.toLowerCase()) <= -1 ) {

				// And if resource content doesn't contain string
				if( $resource.find('div.resource-content').html().toLowerCase().indexOf(search.toLowerCase()) <= -1 ) {

					// Hide resource
					$resource.addClass('hidden-by-search');
				}
			}
		});
	}
};



/* 
 * Globals, initializations
 */

var maxScrollPosition;



/*
 * Events
 */

$(window).load(function() {

	// Start event listeners depending on which page is loaded
	var page = checkWhichPage();
	switch( page ) {
		case 'home':
			initializeSliderState();

			// Handle previous slider button click
			$('button#slider-prev').click(function() {
				var $target = $('div#slider-wrapper ul li.active').prev();
				toGalleryItem($target, maxScrollPosition);
			});
			// Handle next slider button click
			$('button#slider-next').click(function() {
				var $target = $('div#slider-wrapper ul li.active').next();
				toGalleryItem($target, maxScrollPosition);
			});
			break;

		case 'students':
			initializeClassSelectButtons();

			// Handle class year button click
			$('ul#class-select-list li button').click(function() {
				var $selection = $(this);
				editURLText($selection);
				showStudents($selection);
			});
			break;

		case 'resources':
			// Set up tag list
			alphabetizeList($('section#tags-list ul'));
			$('section#tags-list ul').prepend('<li id="all-tag" class="clicked">All</li>');

			// Handle tag click: toggle tag selection, implement tag filtering
			$('section#tags-list ul li').click(function() {
				toggleTag($(this));
				filterByTag();
			});

			// Handle search input entering: implement searching through resources
			$('section#resources-search form button').click(function() {
				searchResources($('section#resources-search form input').val());
			});

			break;
	}
});